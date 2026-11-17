"""
Arma com Arduino - Controlador Python (mira absoluta)

Responsabilidade DESTE programa: receber os dados do Arduino por Bluetooth BLE
(notificações da característica FFE1 do HC-06, via biblioteca Bleak), estimar para
ONDE A PONTA DA ARMA APONTA e colocar o cursor nesse ponto da tela.
Não conhece pinos nem registradores: isso é responsabilidade do .ino.

Cada linha recebida tem o formato:
  L,ax_mg,ay_mg,az_mg,gx_d10,gy_d10,gz_d10,botoes*HH
  Exemplo: L,-513,160,753,107,-15,-7,0*49
(o BLE entrega a linha em pedaços de até ~20 bytes; o programa os junta até o '\n')

Recepção BLE: o Bleak roda em uma thread própria (laço asyncio) e entrega as
notificações por uma fila. Assim o BLE nunca bloqueia o controle do mouse, e uma
queda de conexão é tratada com reconexão automática.

Diagnóstico de queda BLE: quando o enlace cai, o programa imprime linhas "[diag]"
com a duração da conexão, o fluxo de dados antes da queda e os últimos eventos do
programa (botões, fim da calibração, cursor movido...) com o tempo até a queda.
Isso permite descobrir se a queda ocorre junto de algum evento específico.

Como funciona:
  1. Um filtro de orientação (Mahony) junta giroscópio + acelerômetro e mantém
     a orientação 3D do sensor (quaternion).
  2. Calcula-se o vetor da ponta (eixo do cano). Girar o pulso (rodar em torno
     do cano) NÃO muda esse vetor, então o cursor não se mexe.
  3. O ângulo em relação ao centro passa por um filtro anti-tremor (One Euro):
     bem firme quando a mira está parada, rápido quando você move de verdade.
  4. O ângulo vira pixels, com o centro da tela em (0,0), usando o campo de
     visão (FOV_HORIZONTAL_GRAUS).

Por que o sensor na ponta exige cuidado: quando a arma gira em torno do pulso,
a ponta se desloca e o acelerômetro sente essa aceleração além da gravidade
(o tremor de 10 Hz chega a ~0,7 g na ponta). Por isso o acelerômetro corrige
a inclinação só devagar, filtrado e só quando a arma está quase parada.

Calibração: cada etapa só é aprovada pela QUALIDADE dos dados (giroscópio parado,
gravidade perto de 1 g e com direção estável, leituras anormais descartadas), nunca
porque o tempo acabou. O eixo do cano é conferido levantando E abaixando a ponta.
Se a etapa não for aprovada, o programa diz o motivo e continua esperando.

Limite do hardware: o MPU6050 não tem bússola. Cima/baixo é absoluto (gravidade),
mas esquerda/direita deriva devagar. Use F8 para recentralizar quando precisar.

Visualizador 3D: a cada leitura este programa também envia a orientação calculada
por UDP para o próprio computador (127.0.0.1). O arquivo visualizador_3d.py escuta
esses pacotes e mostra o modelo girando. Se ele estiver fechado, nada muda aqui.
Os pacotes também levam "conexao", "calibrando" e "etapa" (campos extras, opcionais).

Teclas (funcionam mesmo com outro programa em foco):
  F7 = recalibrar (refaz a calibração sem reiniciar o programa)
  F8 = recentralizar (a ponta atual passa a ser o centro da tela)
  F9 = pausar / retomar o controle do mouse

Uso:
  python mouse_controller.py                 -> conecta ao HC-06 por BLE (ENDERECO_BLE)
  python mouse_controller.py --address AA:BB:CC:DD:EE:FF -> BLE em outro endereço
  python mouse_controller.py --list          -> lista os dispositivos BLE próximos (marca o HC-06)
  python mouse_controller.py --monitor       -> mostra dados e mira, SEM mover o cursor
  python mouse_controller.py --debug         -> move o cursor e mostra os dados
  python mouse_controller.py --fov 70        -> campo de visão horizontal em graus
  python mouse_controller.py --cutoff 0.8    -> firmeza anti-tremor (menor = mais firme)
  python mouse_controller.py --no-3d         -> não envia dados ao visualizador 3D
"""

import argparse
import asyncio
import importlib.metadata
import json
import math
import platform
import queue
import socket
import sys
import threading
import time
from collections import deque

from bleak import BleakClient, BleakScanner
from pynput import keyboard
from pynput.mouse import Button, Controller

# =====================================================================
# CONFIGURAÇÕES (ajuste aqui)
# =====================================================================
# --- Bluetooth BLE (HC-06 / ZS-040) ---
NOME_BLUETOOTH = "HC-06"    # nome do módulo (usado para localizá-lo se ENDERECO_BLE for None)
ENDERECO_BLE = "EE:52:CB:4B:54:7C"   # endereço do HC-06 (None = procurar pelo nome)
UUID_SERVICO_BLE = "0000ffe0-0000-1000-8000-00805f9b34fb"   # serviço (informativo)
UUID_CARACTERISTICA_BLE = "0000ffe1-0000-1000-8000-00805f9b34fb"  # FFE1: write + notify
TIMEOUT_BUSCA_BLE_S = 10.0  # quanto tempo procurar o módulo a cada tentativa
TIMEOUT_CONEXAO_BLE_S = 15.0  # tempo máximo para abrir a conexão

# --- Sensibilidade ---
# Ângulo horizontal que cobre a tela inteira (da borda esquerda à direita).
# MAIOR = cursor MENOS sensível (menos pixels por grau); menor = mais sensível.
# Valores práticos: 50 a 80. Menor que 40 amplifica muito o tremor da mão.
FOV_HORIZONTAL_GRAUS = 60.0
INVERTER_X = False          # True se o cursor for para o lado errado (esq/dir)
INVERTER_Y = False          # True se o cursor for para o lado errado (cima/baixo)

# --- Visualizador 3D (visualizador_3d.py) ---
PORTA_VISUALIZADOR = 49555  # porta UDP local para onde os dados são enviados (None = desligado)

# --- Anti-tremor (filtro One Euro, aplicado nos ângulos em graus) ---
FILTRO_CUTOFF_HZ = 1.0      # firmeza com a mira parada: MENOR = mais firme (e mais lento)
FILTRO_BETA = 0.04          # MAIOR = responde mais rápido em movimentos grandes
FILTRO_D_CUTOFF_HZ = 1.0    # filtro da velocidade (mantém o tremor de ~10 Hz de fora)
ZONA_ESTAVEL_PX = 2         # o cursor só se move se a mira mudar pelo menos isso (px)
TRAVA_GATILHO_MS = 80       # congela o cursor por esse tempo ao apertar o botão 1
                            # (o "tranco" do gatilho não desloca o tiro). 0 = desligado

# Eixo do cano no sensor. None = descobrir na calibração (levantando a ponta).
# Depois de descobrir, copie o valor que o script mostrar, por exemplo:
# BARREL_AXIS = (0.99, 0.02, -0.05)   e o passo de levantar a ponta será pulado.
BARREL_AXIS = None

# --- Filtro de orientação ---
GANHO_ACEL = 0.3            # quanto o acelerômetro corrige a inclinação (menor = mais estável)
GANHO_ACEL_INICIAL = 10.0   # correção rápida usada só na calibração
ACEL_TAU_S = 0.2            # suavização do acelerômetro antes de corrigir (tira o tremor)
GIRO_MAX_CORRECAO_DPS = 60.0  # acima dessa velocidade de giro o acelerômetro é ignorado
ACEL_MIN_G = 0.85           # acelerômetro só é confiável perto de 1 g
ACEL_MAX_G = 1.15

# --- Calibração e correção de drift ---
CALIBRAR_OFFSET_APOIADO = True     # passo 1: apoiar a arma numa superfície (offset do giro preciso)
AMOSTRAS_CALIBRACAO = 150          # ~1,5 s a 100 quadros/s
LIMITE_APOIADO_DPS = 3.0           # variação máxima do giro com a arma apoiada
LIMITE_NA_MAO_DPS = 40.0           # variação máxima do giro com a arma na mão (tremor)
LEVANTE_MIN_GRAUS = 15.0           # a ponta precisa subir pelo menos isso na calibração
LEVANTE_MAX_GRAUS = 85.0
REPOUSO_GIRO_DPS = 1.0             # abaixo disso o dispositivo é considerado parado
REPOUSO_ACEL_G = 0.03              # tolerância da aceleração em repouso (|a| ~ 1 g)
REPOUSO_TEMPO_S = 0.5              # tempo parado antes de corrigir o offset do giro
ADAPTACAO_OFFSET = 0.02            # velocidade da correção do offset (por quadro)

# --- Qualidade da calibração (uma etapa só é aprovada se os dados forem bons) ---
# ATENÇÃO: estes limites ainda não foram validados no seu hardware. Se a calibração
# nunca for aprovada, o programa avisa o motivo; só então afrouxe o limite indicado.
LIMITE_DIRECAO_APOIADO_GRAUS = 1.0 # variação máxima da direção da gravidade (apoiada)
LIMITE_DIRECAO_NA_MAO_GRAUS = 4.0  # idem, com a arma na mão
CALIB_ACEL_MIN_G = 0.90            # |a| aceito como "parado" (a gravidade vale 1 g)
CALIB_ACEL_MAX_G = 1.10
CALIB_FRACAO_DESCARTE_MAX = 0.15   # até 15% da janela pode ser descartada como "anormal"
CALIB_FATOR_OUTLIER = 5.0          # anormal = desvio maior que 5x o ruído típico (MAD)
CALIB_PISO_GIRO_DPS = 0.3          # ruído mínimo considerado no giro (resolução do sensor)
CALIB_PISO_ACEL_G = 0.01           # idem, no acelerômetro
OFFSET_GIRO_MAX_DPS = 25.0         # offset acima disso é suspeito (MPU6050: ~±20 °/s)
VALIDAR_CANO_ABAIXANDO = True      # passo extra: abaixar a ponta e conferir o eixo do cano
TOLERANCIA_EIXO_CANO_GRAUS = 8.0   # máximo de diferença entre o eixo (subindo) e (descendo)
CALIB_AVISO_DEMORA_S = 20.0        # depois disso o programa explica o que verificar

# --- Comunicação ---
TIMEOUT_SEM_DADOS_S = 2.0   # sem dados por esse tempo = comunicação perdida
INTERVALO_RECONEXAO_S = 2.0
TIMEOUT_BOTOES_S = 0.5      # sem dados por esse tempo: solta os botões do mouse (não ficam presos)
IDADE_MAX_DADOS_S = 0.5     # dado que ficou mais tempo que isso na fila é velho e é descartado

# --- Diagnóstico BLE ---
DIAG_SILENCIO_ANTES_QUEDA_S = 1.0   # se o fluxo parou antes da queda por mais que isso, o diag avisa
DIAG_MARCOS_JANELA_S = 20.0         # quantos segundos de eventos antes da queda entram no diag
TIMEOUT_STOP_NOTIFY_S = 3.0         # limite para parar as notificações ao encerrar uma conexão


class ErroComunicacao(Exception):
    """Falha de comunicação com o Arduino (sem dados, porta perdida)."""


# =====================================================================
# UTILITÁRIOS
# =====================================================================
def listar_dispositivos_ble():
    """Procura dispositivos BLE por alguns segundos e lista (marca o HC-06)."""
    print("Procurando dispositivos BLE (8 s)...")
    try:
        dispositivos = asyncio.run(BleakScanner.discover(timeout=8.0))
    except Exception as erro:
        print(f"Não foi possível procurar dispositivos BLE: {erro}\n"
              "  Verifique se o Bluetooth do Windows está ligado.")
        return
    if not dispositivos:
        print("Nenhum dispositivo BLE encontrado.")
        return
    print("Dispositivos BLE encontrados:")
    for d in sorted(dispositivos, key=lambda d: (d.name or "~")):
        eh_hc = (ENDERECO_BLE and d.address.upper() == ENDERECO_BLE.upper()) or \
                (d.name or "").lower() == NOME_BLUETOOTH.lower()
        marca = f"   <-- {NOME_BLUETOOTH}" if eh_hc else ""
        print(f"  {d.address}  {d.name or '(sem nome)'}{marca}")


def tamanho_da_tela():
    """Resolução real da tela principal, em pixels."""
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(2)  # evita escala do Windows
        except Exception:
            ctypes.windll.user32.SetProcessDPIAware()
        usuario = ctypes.windll.user32
        return usuario.GetSystemMetrics(0), usuario.GetSystemMetrics(1)
    except Exception:
        import tkinter
        raiz = tkinter.Tk()
        largura, altura = raiz.winfo_screenwidth(), raiz.winfo_screenheight()
        raiz.destroy()
        return largura, altura


def _interpretar_csv(texto):
    """Linha do Bluetooth -> dicionário igual ao do JSON da USB. None se inválida.
      L,ax_mg,ay_mg,az_mg,gx_d10,gy_d10,gz_d10,botoes*HH   (HH = XOR dos caracteres antes do '*')
      S,pronto   |   E,mpu_nao_responde"""
    corpo, _, soma = texto.partition("*")
    partes = corpo.split(",")
    tipo = partes[0]
    if tipo == "S":
        return {"status": partes[1] if len(partes) > 1 else ""}
    if tipo == "E":
        return {"erro": partes[1] if len(partes) > 1 else "desconhecido"}
    if tipo != "L" or len(partes) != 8 or len(soma) != 2:
        return None
    try:
        if int(soma, 16) != _xor_texto(corpo):
            return None  # byte perdido/corrompido no Bluetooth
        v = [int(x) for x in partes[1:]]
    except ValueError:
        return None
    return {"ax": v[0] / 1000.0, "ay": v[1] / 1000.0, "az": v[2] / 1000.0,
            "gx": v[3] / 10.0, "gy": v[4] / 10.0, "gz": v[5] / 10.0,
            "b1": v[6] & 1, "b2": (v[6] >> 1) & 1}


def _xor_texto(texto):
    resultado = 0
    for c in texto:
        resultado ^= ord(c)
    return resultado


def interpretar_linha(linha_bytes):
    """Converte uma linha (JSON da USB ou CSV do Bluetooth) em dicionário. None se inválida."""
    texto = linha_bytes.decode("ascii", errors="ignore").strip()
    if texto[:2] in ("L,", "S,", "E,"):
        return _interpretar_csv(texto)
    if not texto.startswith("{"):
        return None
    try:
        dados = json.loads(texto)
    except ValueError:
        return None  # linha cortada/corrompida (comum logo após abrir a porta)
    return dados if isinstance(dados, dict) else None


def extrair_dados(quadro):
    """Retorna (aceleração[3], giro[3], botao1, botao2) ou None."""
    try:
        aceleracao = (float(quadro["ax"]), float(quadro["ay"]), float(quadro["az"]))
        giro = (float(quadro["gx"]), float(quadro["gy"]), float(quadro["gz"]))
        return aceleracao, giro, int(quadro["b1"]) == 1, int(quadro["b2"]) == 1
    except (KeyError, TypeError, ValueError):
        return None


def normalizar_angulo(angulo):
    """Leva um ângulo em radianos para o intervalo [-pi, pi]."""
    return (angulo + math.pi) % (2.0 * math.pi) - math.pi


def norma(vetor):
    return math.sqrt(sum(v * v for v in vetor))


def unitario(vetor):
    modulo = norma(vetor)
    return tuple(v / modulo for v in vetor)


def mediana(valores):
    ordenados = sorted(valores)
    n = len(ordenados)
    meio = n // 2
    return ordenados[meio] if n % 2 else (ordenados[meio - 1] + ordenados[meio]) / 2.0


def desvio_robusto(valores, centro=None):
    """Desvio padrão estimado pela MAD (mediana dos desvios absolutos): quase não é
    afetado por alguns valores muito fora do normal."""
    if centro is None:
        centro = mediana(valores)
    return 1.4826 * mediana([abs(v - centro) for v in valores])


def angulo_entre(u, v):
    """Ângulo (graus) entre dois vetores unitários."""
    cos_ang = max(-1.0, min(1.0, sum(a * b for a, b in zip(u, v))))
    return math.degrees(math.acos(cos_ang))


def avaliar_janela_parada(janela, limite_dps, limite_direcao_graus):
    """Decide se uma janela de amostras [(aceleração, giro), ...] serve para calibrar.

    Uma etapa só é aprovada pela QUALIDADE dos dados, nunca porque o tempo passou:
      1. leituras anormais (picos, bytes ruins) são descartadas; se forem muitas, reprova;
      2. giroscópio parado: variação pico-a-pico e tendência (movimento lento) dentro do limite;
      3. acelerômetro: |a| perto de 1 g e direção da gravidade estável (em graus).
    Retorna (aprovada, motivo, resultado). O resultado traz a aceleração média e o
    offset do giroscópio (mediana) das amostras boas, mais as métricas de qualidade."""
    n = len(janela)
    acel = [a for a, _ in janela]
    giro = [g for _, g in janela]

    med_g = [mediana([g[i] for g in giro]) for i in range(3)]
    med_a = [mediana([a[i] for a in acel]) for i in range(3)]
    lim_g = [CALIB_FATOR_OUTLIER * max(desvio_robusto([g[i] for g in giro], med_g[i]),
                                       CALIB_PISO_GIRO_DPS) for i in range(3)]
    lim_a = [CALIB_FATOR_OUTLIER * max(desvio_robusto([a[i] for a in acel], med_a[i]),
                                       CALIB_PISO_ACEL_G) for i in range(3)]
    bons = [k for k in range(n)
            if all(abs(giro[k][i] - med_g[i]) <= lim_g[i] and
                   abs(acel[k][i] - med_a[i]) <= lim_a[i] for i in range(3))]
    descartadas = n - len(bons)
    if descartadas > CALIB_FRACAO_DESCARTE_MAX * n:
        return False, "leituras instáveis (muitos valores anormais)", None
    giro = [giro[k] for k in bons]
    acel = [acel[k] for k in bons]

    pico = max(max(g[i] for g in giro) - min(g[i] for g in giro) for i in range(3))
    if pico > limite_dps:
        return False, "o giroscópio ainda indica movimento", None
    metade = len(giro) // 2
    tendencia = max(abs(mediana([g[i] for g in giro[:metade]]) -
                        mediana([g[i] for g in giro[metade:]])) for i in range(3))
    if tendencia > limite_dps / 2.0:
        return False, "movimento lento detectado (o giro muda ao longo da janela)", None

    normas = [norma(a) for a in acel]
    if min(normas) < CALIB_ACEL_MIN_G or max(normas) > CALIB_ACEL_MAX_G:
        return False, "aceleração fora de 1 g (sensor sacudido ou leitura inválida)", None
    media_a = tuple(sum(a[i] for a in acel) / len(acel) for i in range(3))
    if norma(media_a) < 0.5:
        return False, "inclinação instável", None
    direcao = unitario(media_a)
    espalhamento = max(angulo_entre(unitario(a), direcao) for a in acel)
    if espalhamento > limite_direcao_graus:
        return False, "inclinação instável (arma se mexendo ou mão tremendo)", None

    return True, "", {
        "acel": media_a,
        "giro": tuple(mediana([g[i] for g in giro]) for i in range(3)),
        "pico_giro": pico, "espalhamento": espalhamento,
        "descartadas": descartadas, "n": len(giro),
    }


def calcular_eixo_do_cano(acel_centro, acel_levantada):
    """Descobre o eixo do cano no sensor.

    Com a arma parada, o acelerômetro mede a direção "para cima" no corpo do
    sensor. Ao LEVANTAR a ponta sem torcer o pulso, essa direção "para cima" se
    desloca na direção do cano. Então o cano é a parte da mudança que é
    perpendicular à direção original.

    Retorna (eixo_unitario, angulo_levantado_em_graus).
    """
    u0 = unitario(acel_centro)
    u1 = unitario(acel_levantada)
    cos_ang = max(-1.0, min(1.0, sum(a * b for a, b in zip(u0, u1))))
    angulo = math.degrees(math.acos(cos_ang))
    perpendicular = tuple(u1[i] - cos_ang * u0[i] for i in range(3))
    if norma(perpendicular) < 1e-6:
        return None, angulo
    return unitario(perpendicular), angulo


class FiltroOneEuro:
    """Suavização adaptativa. Com a mira quase parada o corte é baixo (firme, tira
    o tremor); quanto mais rápido o movimento, mais o corte sobe (sem atraso)."""

    def __init__(self, cutoff_min, beta, cutoff_derivada):
        self.cutoff_min = cutoff_min
        self.beta = beta
        self.cutoff_derivada = cutoff_derivada
        self.x_anterior = None
        self.dx_anterior = 0.0

    def reiniciar(self):
        self.x_anterior = None
        self.dx_anterior = 0.0

    @staticmethod
    def _alfa(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, dt):
        if self.x_anterior is None:
            self.x_anterior = x
            self.dx_anterior = 0.0
            return x
        dx = (x - self.x_anterior) / dt
        self.dx_anterior += self._alfa(self.cutoff_derivada, dt) * (dx - self.dx_anterior)
        cutoff = self.cutoff_min + self.beta * abs(self.dx_anterior)
        self.x_anterior += self._alfa(cutoff, dt) * (x - self.x_anterior)
        return self.x_anterior


# =====================================================================
# RECEPTOR BLE (Bleak, assíncrono, em thread própria)
# =====================================================================
class ReceptorBLE:
    """Conecta ao HC-06 por BLE, assina as notificações da FFE1 e reconecta sozinho.

    Roda um laço asyncio em uma thread separada para nunca bloquear o controle do mouse.
    Tudo o que acontece chega ao programa principal pela fila `fila`, em tuplas:
      ("dados", (época, t, bytes))  pedaço recebido por notificação, com o número da conexão
                              (época) e o instante de chegada (t, time.monotonic())
      ("conectado", (época, nome))  conexão aberta e notificações ativas
      ("desconectado", texto) conexão perdida (o receptor já começa a reconectar); quando a
                              queda veio do enlace, o texto inclui linhas "[diag]"
      ("aviso", texto)        problema ao procurar/conectar (tentará de novo)

    Diagnóstico: os instantes da última notificação e da queda são medidos DENTRO da thread
    BLE (não dependem do atraso da fila), e o programa principal registra eventos com
    marcar() para que a queda possa ser comparada com o que estava acontecendo.
    """

    MOTIVO_QUEDA = ("O enlace BLE com o HC-06 caiu: o Bleak/Windows informou desconexão "
                    "(módulo reiniciou ou perdeu energia, saiu de alcance, ou a sessão foi encerrada).")

    def __init__(self, endereco, nome, uuid_caracteristica):
        self.endereco = endereco
        self.nome = nome
        self.uuid = uuid_caracteristica
        self.fila = queue.Queue()
        self.conectado = threading.Event()
        self._thread = None
        self._loop = None
        self._forcar = None      # asyncio.Event: pede para derrubar e reconectar
        self._parar = None       # asyncio.Event: pede para encerrar de vez
        self._pronto = threading.Event()
        self._epoca = 0          # número da conexão atual: dado de outra conexão é descartado
        # --- diagnóstico ---
        self.marcos = deque(maxlen=40)   # (instante, texto): eventos do programa principal
        self._t_conexao = None
        self._t_ultimo_dado = None
        self._t_queda = None
        self._bytes_total = 0
        self._chunks_total = 0
        self._recentes = deque(maxlen=600)  # (instante, nº de bytes) das últimas notificações

    # ----- controle (chamado pela thread principal) -----
    def iniciar(self):
        self._thread = threading.Thread(target=self._rodar, name="ble", daemon=True)
        self._thread.start()
        self._pronto.wait(timeout=5.0)

    def forcar_reconexao(self):
        """Derruba a conexão atual (se houver) e reconecta, p.ex. quando ela parou de enviar."""
        self._chamar_no_laco(lambda: self._forcar and self._forcar.set())

    def parar(self):
        self._chamar_no_laco(lambda: self._parar and self._parar.set())
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    def marcar(self, texto):
        """Registra um evento do programa (com o instante) para o diagnóstico de queda."""
        self.marcos.append((time.monotonic(), texto))

    def resumo(self):
        """Uma linha curta com o estado do fluxo BLE (usada nas mensagens de debug)."""
        if not self.conectado.is_set():
            return "ble: desconectado"
        agora = time.monotonic()
        taxa = self._taxa(agora)
        ult = "-" if self._t_ultimo_dado is None else f"{(agora - self._t_ultimo_dado) * 1000:.0f} ms"
        return (f"ble: {taxa:.0f} B/s, {self._chunks_total} notif., último dado há {ult}, "
                f"fila={self.fila.qsize()}")

    def _chamar_no_laco(self, funcao):
        loop = self._loop
        if loop is None:
            return
        try:
            loop.call_soon_threadsafe(funcao)
        except RuntimeError:
            pass  # laço já encerrado

    def _emitir(self, tipo, valor=None):
        self.fila.put((tipo, valor))

    # ----- diagnóstico -----
    def _taxa(self, ate, janela_s=2.0):
        """Bytes por segundo recebidos na janela que termina em `ate`."""
        total = sum(n for t, n in list(self._recentes) if ate - janela_s <= t <= ate)
        return total / janela_s

    def _diagnostico_queda(self, t_queda):
        """Linhas [diag] explicando o que se sabe sobre a queda (fatos medidos, sem adivinhar)."""
        linhas = []
        if self._t_conexao is not None:
            linhas.append(f"  [diag] conexão durou {t_queda - self._t_conexao:.1f} s; "
                          f"{self._chunks_total} notificações, {self._bytes_total} bytes recebidos.")
        if self._t_ultimo_dado is None:
            linhas.append("  [diag] NENHUMA notificação chegou nesta conexão antes da queda.")
        else:
            silencio = t_queda - self._t_ultimo_dado
            taxa = self._taxa(self._t_ultimo_dado)
            linhas.append(f"  [diag] o último dado chegou {silencio * 1000:.0f} ms antes da queda; "
                          f"fluxo nos 2 s anteriores a ele: {taxa:.0f} B/s.")
            if silencio > DIAG_SILENCIO_ANTES_QUEDA_S:
                linhas.append("  [diag] indício: o fluxo PAROU antes da queda (Arduino/módulo "
                              "travou ou deixou de enviar; o enlace caiu depois por timeout).")
            else:
                linhas.append("  [diag] indício: o fluxo seguia normal até o instante da queda "
                              "(queda abrupta do enlace: energia/reset do módulo, alcance ou Windows).")
        recentes = [(t, txt) for t, txt in list(self.marcos)
                    if t_queda - DIAG_MARCOS_JANELA_S <= t <= t_queda]
        if recentes:
            linhas.append("  [diag] eventos antes da queda:")
            for t, txt in recentes:
                linhas.append(f"           {t_queda - t:6.2f} s antes: {txt}")
        else:
            linhas.append(f"  [diag] nenhum evento do programa nos {DIAG_MARCOS_JANELA_S:.0f} s "
                          "anteriores à queda.")
        return "\n".join(linhas)

    # ----- thread do BLE -----
    def _rodar(self):
        try:
            asyncio.run(self._principal())
        except Exception as erro:  # nunca deixa a thread morrer em silêncio
            self._emitir("aviso", f"Falha inesperada no BLE: {erro!r}")

    def _ao_notificar(self, _caracteristica, dados):
        # chamado pelo Bleak a cada notificação (cada uma traz poucos bytes)
        agora = time.monotonic()
        self._t_ultimo_dado = agora
        self._bytes_total += len(dados)
        self._chunks_total += 1
        self._recentes.append((agora, len(dados)))
        self._emitir("dados", (self._epoca, agora, bytes(dados)))

    async def _principal(self):
        self._loop = asyncio.get_running_loop()
        self._forcar = asyncio.Event()
        self._parar = asyncio.Event()
        self._pronto.set()
        falhas = 0
        while not self._parar.is_set():
            conectou = False
            try:
                conectou = await self._conectar_e_receber()
            except Exception as erro:  # BleakError, OSError, TimeoutError...
                self._emitir("aviso", f"Falha na conexão BLE: {erro!r}")
            falhas = 0 if conectou else falhas + 1
            # espera crescente (até 10 s) se as tentativas seguem falhando
            espera = min(INTERVALO_RECONEXAO_S * max(1, falhas), 10.0)
            try:  # intervalo entre tentativas (interrompível pelo encerramento)
                await asyncio.wait_for(self._parar.wait(), timeout=espera)
            except asyncio.TimeoutError:
                pass

    async def _localizar(self):
        if self.endereco:
            return await BleakScanner.find_device_by_address(
                self.endereco, timeout=TIMEOUT_BUSCA_BLE_S)
        return await BleakScanner.find_device_by_name(
            self.nome, timeout=TIMEOUT_BUSCA_BLE_S)

    async def _conectar_e_receber(self):
        """Uma conexão completa. Retorna True se chegou a conectar, False se não achou o módulo."""
        dispositivo = await self._localizar()
        if dispositivo is None:
            alvo = self.endereco or self.nome
            self._emitir("aviso", f"{NOME_BLUETOOTH} ({alvo}) não encontrado. Confira se está "
                                  "ligado, ao alcance e sem outro aparelho conectado a ele "
                                  "(use --list para ver os dispositivos BLE).")
            return False

        loop = asyncio.get_running_loop()
        desconectou = asyncio.Event()
        self._t_queda = None

        def ao_desconectar(_cliente):
            # Instante medido aqui, no momento em que o Bleak avisa (sem depender da fila).
            if self._t_queda is None:
                self._t_queda = time.monotonic()
            loop.call_soon_threadsafe(desconectou.set)

        self._forcar.clear()
        cliente = BleakClient(dispositivo, disconnected_callback=ao_desconectar,
                              timeout=TIMEOUT_CONEXAO_BLE_S)
        await cliente.connect()
        try:
            self._epoca += 1  # tudo o que chegar daqui em diante pertence a esta conexão
            self._bytes_total = 0
            self._chunks_total = 0
            self._t_ultimo_dado = None
            self._recentes.clear()
            self._t_conexao = time.monotonic()
            self.marcar("conexão BLE aberta")
            await cliente.start_notify(self.uuid, self._ao_notificar)
            self.conectado.set()
            self._emitir("conectado", (self._epoca,
                                       f"{dispositivo.name or self.nome} ({dispositivo.address})"))
            motivo = await self._esperar_fim(desconectou)
        finally:
            self.conectado.clear()
            if cliente.is_connected:
                # encerramento limpo: desliga as notificações antes de fechar o enlace
                try:
                    await asyncio.wait_for(cliente.stop_notify(self.uuid),
                                           timeout=TIMEOUT_STOP_NOTIFY_S)
                except Exception as erro:
                    self._emitir("aviso", f"stop_notify não concluiu: {erro!r}")
            try:
                await cliente.disconnect()
            except Exception:
                pass
        if motivo == self.MOTIVO_QUEDA and self._t_queda is not None:
            motivo = motivo + "\n" + self._diagnostico_queda(self._t_queda)
        self._emitir("desconectado", motivo)
        return True

    async def _esperar_fim(self, desconectou):
        t_desc = asyncio.ensure_future(desconectou.wait())
        t_forca = asyncio.ensure_future(self._forcar.wait())
        t_parar = asyncio.ensure_future(self._parar.wait())
        feitas, pendentes = await asyncio.wait({t_desc, t_forca, t_parar},
                                               return_when=asyncio.FIRST_COMPLETED)
        for tarefa in pendentes:
            tarefa.cancel()
        if t_desc in feitas:
            return self.MOTIVO_QUEDA
        if t_forca in feitas:
            return "Reconexão solicitada pelo programa."
        return "Encerrando."


# =====================================================================
# CONTROLADOR
# =====================================================================
class ArmaArduino:
    def __init__(self, args):
        # Bluetooth BLE: endereço do HC-06 (--address sobrescreve ENDERECO_BLE)
        self.ble = ReceptorBLE(args.address or ENDERECO_BLE, NOME_BLUETOOTH,
                               UUID_CARACTERISTICA_BLE)
        # Bluetooth chega em ritmo menor (~25 quadros/s): ajusta tempos que supunham ~100/s
        self.dt_max = 0.15
        self.amostras_calibracao = 40
        self.monitor = args.monitor      # só mostra dados, não controla o mouse
        self.debug = args.debug or args.monitor

        # Tela e campo de visão. O FOV vertical segue a proporção da tela.
        self.largura, self.altura = tamanho_da_tela()
        fov = args.fov if args.fov is not None else FOV_HORIZONTAL_GRAUS
        fov = min(max(fov, 5.0), 120.0)
        self.tan_meia_h = math.tan(math.radians(fov / 2.0))
        self.tan_meia_v = self.tan_meia_h * self.altura / self.largura

        # Filtro anti-tremor (um para cada eixo, em graus)
        cutoff = args.cutoff if args.cutoff is not None else FILTRO_CUTOFF_HZ
        cutoff = max(cutoff, 0.1)
        self.filtro_x = FiltroOneEuro(cutoff, FILTRO_BETA, FILTRO_D_CUTOFF_HZ)
        self.filtro_y = FiltroOneEuro(cutoff, FILTRO_BETA, FILTRO_D_CUTOFF_HZ)

        # Envio de dados para o visualizador 3D (UDP local, sem esperar resposta)
        self._udp = None
        self._destino_3d = None
        if PORTA_VISUALIZADOR and not args.no_3d:
            self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._udp.setblocking(False)  # o visualizador nunca pode segurar o controlador
            self._destino_3d = ("127.0.0.1", PORTA_VISUALIZADOR)

        self.mouse = Controller()
        self.esq_pressionado = False
        self.dir_pressionado = False
        self.trava_ate = 0.0           # cursor congelado até este instante (gatilho)

        # Teclas globais
        self.pedido_recentro = False
        self.ativo = True
        self._listener = None

        # Estado do filtro de orientação
        self.q = (1.0, 0.0, 0.0, 0.0)          # orientação (w, x, y, z)
        self.barril = (1.0, 0.0, 0.0)          # eixo do cano no sensor
        self.offset_giro = [0.0, 0.0, 0.0]     # offset do giroscópio (°/s)
        self.acel_filtrada = None
        self.yaw0 = 0.0
        self.pitch0 = 0.0
        self.t_anterior = None
        self.dt = 0.01
        self.tempo_repouso = 0.0
        self._ultimo_px = None

        self._ultimo_aviso = None
        self._ultimo_debug = 0.0

        # Estado compartilhado com o visualizador 3D e com a calibração
        self.estado_conexao = "procurando"
        self.calibrando = False
        self.etapa_calibracao = ""
        self.pedido_recalibrar = False
        self._epoca_atual = 0                    # conexão BLE cujos dados são aceitos
        self._ult_viz = (0.0, 0.0, 0.0, 0.0, False, False)
        self._ultimo_envio_viz = 0.0
        self._calib_motivo = None
        self._calib_t_motivo = 0.0
        self._primeiro_quadro_ativo = False      # diagnóstico: marca o 1º quadro após a calibração

    # ---------- avisos ----------
    def _aviso(self, msg):
        """Imprime a mensagem apenas se for diferente da última (evita spam)."""
        if msg != self._ultimo_aviso:
            print(msg)
            self._ultimo_aviso = msg

    # ---------- teclas globais ----------
    def _iniciar_teclas(self):
        def ao_pressionar(tecla):
            if tecla == keyboard.Key.f7:
                self.pedido_recalibrar = True
                self.ble.marcar("F7 (recalibrar)")
                print("Recalibração solicitada...")
            elif tecla == keyboard.Key.f8:
                self.pedido_recentro = True
                self.ble.marcar("F8 (recentralizar)")
            elif tecla == keyboard.Key.f9:
                self.ativo = not self.ativo
                self.ble.marcar("F9 (controle do mouse " + ("retomado)" if self.ativo else "pausado)"))
                print("Controle do mouse RETOMADO." if self.ativo else "Controle do mouse PAUSADO.")

        self._listener = keyboard.Listener(on_press=ao_pressionar)
        self._listener.daemon = True
        self._listener.start()

    def _parar_teclas(self):
        if self._listener is not None:
            self._listener.stop()
            self._listener = None

    # ---------- leitura BLE ----------
    def _aguardar_conexao(self):
        """Espera o receptor BLE abrir a conexão. Descarta dados antigos de conexões anteriores."""
        self.estado_conexao = "procurando"
        self._enviar_estado_visualizador(forcar=True)
        self._aviso(f"Procurando {NOME_BLUETOOTH} por Bluetooth BLE...")
        while True:
            try:
                tipo, valor = self.ble.fila.get(timeout=0.2)
            except queue.Empty:
                self._enviar_estado_visualizador()
                continue
            if tipo == "conectado":
                self._epoca_atual, nome = valor
                self.estado_conexao = "conectado"
                self._enviar_estado_visualizador(forcar=True)
                print(f"Conectado a {nome} por BLE.")
                self._ultimo_aviso = None
                return
            if tipo == "aviso":
                self._aviso(valor)
            # "dados" e "desconectado" velhos são descartados

    def _ler_quadros(self):
        """Gerador que entrega cada quadro recebido do Arduino (via notificações BLE).
        Nunca entrega dado de outra conexão nem dado atrasado; se o sinal some, solta os
        botões do mouse rápido e, depois de TIMEOUT_SEM_DADOS_S, declara a comunicação perdida."""
        buffer = bytearray()
        ultimo_dado = time.monotonic()
        silencio_tratado = False
        while True:
            try:
                tipo, valor = self.ble.fila.get(timeout=0.05)  # espera até 50 ms por dados
            except queue.Empty:
                tipo, valor = None, None
            agora = time.monotonic()
            if tipo == "dados":
                epoca, t_rx, pedaco = valor
                if epoca != self._epoca_atual:
                    continue  # sobra de uma conexão anterior: nunca é processada
                ultimo_dado = agora
                silencio_tratado = False
                if agora - t_rx > IDADE_MAX_DADOS_S:
                    buffer.clear()
                    continue  # ficou tempo demais na fila: é dado velho, usa só o recente
                buffer.extend(pedaco)
                while True:
                    fim = buffer.find(b"\n")
                    if fim < 0:
                        break
                    linha = bytes(buffer[:fim])
                    del buffer[:fim + 1]
                    quadro = interpretar_linha(linha)
                    if quadro is not None:
                        yield quadro
                if len(buffer) > 2048:  # lixo sem quebra de linha: descarta
                    buffer.clear()
            elif tipo == "desconectado":
                raise ErroComunicacao(valor)
            elif tipo == "aviso":
                self._aviso(valor)  # antes era ignorado em silêncio durante a sessão
            elif tipo is None:
                silencio = agora - ultimo_dado
                if silencio > TIMEOUT_BOTOES_S and not silencio_tratado:
                    # sinal sumiu: não deixa botão do mouse preso nem usa dt gigante depois
                    silencio_tratado = True
                    self.soltar_botoes()
                    self.t_anterior = None
                if silencio > TIMEOUT_SEM_DADOS_S:
                    raise ErroComunicacao(
                        f"Sem dados do Arduino há mais de {TIMEOUT_SEM_DADOS_S:.0f} s."
                    )

    # ---------- filtro de orientação (Mahony, 6 eixos) ----------
    def _filtro(self, aceleracao, giro_rad, dt, kp):
        """Atualiza o quaternion q com o giro (rad/s) e corrige com a gravidade."""
        q0, q1, q2, q3 = self.q
        gx, gy, gz = giro_rad

        ax, ay, az = aceleracao
        modulo = norma(aceleracao)
        if kp > 0.0 and ACEL_MIN_G < modulo < ACEL_MAX_G:
            ax, ay, az = ax / modulo, ay / modulo, az / modulo
            # direção da gravidade ("para cima") estimada pela orientação atual
            vx = 2.0 * (q1 * q3 - q0 * q2)
            vy = 2.0 * (q0 * q1 + q2 * q3)
            vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
            # erro = medido x estimado; realimenta no giroscópio
            gx += kp * (ay * vz - az * vy)
            gy += kp * (az * vx - ax * vz)
            gz += kp * (ax * vy - ay * vx)

        # integra: q' = q + 0.5 * q * (0, giro) * dt
        gx *= 0.5 * dt
        gy *= 0.5 * dt
        gz *= 0.5 * dt
        n0 = q0 + (-q1 * gx - q2 * gy - q3 * gz)
        n1 = q1 + (q0 * gx + q2 * gz - q3 * gy)
        n2 = q2 + (q0 * gy - q1 * gz + q3 * gx)
        n3 = q3 + (q0 * gz + q1 * gy - q2 * gx)
        modulo_q = math.sqrt(n0 * n0 + n1 * n1 + n2 * n2 + n3 * n3)
        self.q = (n0 / modulo_q, n1 / modulo_q, n2 / modulo_q, n3 / modulo_q)

    def _direcao(self):
        """Para onde a ponta aponta no mundo: retorna (yaw, pitch) em radianos.
        Girar em torno do cano não altera o vetor da ponta, portanto nem yaw nem pitch."""
        w, x, y, z = self.q
        bx, by, bz = self.barril
        # vetor do cano no mundo = R(q) * barril  (eixo z do mundo = para cima)
        wx = (1.0 - 2.0 * (y * y + z * z)) * bx + 2.0 * (x * y - w * z) * by + 2.0 * (x * z + w * y) * bz
        wy = 2.0 * (x * y + w * z) * bx + (1.0 - 2.0 * (x * x + z * z)) * by + 2.0 * (y * z - w * x) * bz
        wz = 2.0 * (x * z - w * y) * bx + 2.0 * (y * z + w * x) * by + (1.0 - 2.0 * (x * x + y * y)) * bz
        yaw = math.atan2(wy, wx)
        pitch = math.asin(max(-1.0, min(1.0, wz)))
        return yaw, pitch

    def _angulos_relativos(self):
        """Ângulos (graus) da ponta em relação ao centro: (horizontal, vertical).
        Horizontal positivo = direita; vertical positivo = cima."""
        yaw, pitch = self._direcao()
        dyaw = normalizar_angulo(yaw - self.yaw0)
        dpitch = pitch - self.pitch0
        # girar para a direita diminui o yaw (sentido horário visto de cima)
        ang_x = dyaw if INVERTER_X else -dyaw
        ang_y = -dpitch if INVERTER_Y else dpitch
        return math.degrees(ang_x), math.degrees(ang_y)

    def _para_pixels(self, ang_x, ang_y):
        """Ângulos (graus) -> pixels a partir do centro da tela (x direita, y cima)."""
        ang_x = max(-80.0, min(80.0, ang_x))  # evita tan() explodir
        ang_y = max(-80.0, min(80.0, ang_y))
        x = math.tan(math.radians(ang_x)) / self.tan_meia_h * (self.largura / 2.0)
        y = math.tan(math.radians(ang_y)) / self.tan_meia_v * (self.altura / 2.0)
        x = max(-self.largura / 2.0, min(self.largura / 2.0, x))
        y = max(-self.altura / 2.0, min(self.altura / 2.0, y))
        return x, y

    def _definir_centro(self):
        """A direção atual da ponta passa a ser o centro da tela (0, 0)."""
        self.yaw0, self.pitch0 = self._direcao()
        self.filtro_x.reiniciar()
        self.filtro_y.reiniciar()
        if not self.monitor:
            centro = (self.largura // 2, self.altura // 2)
            self.mouse.position = centro
            self._ultimo_px = centro
            self.ble.marcar("cursor movido ao centro da tela")

    # ---------- calibração guiada ----------
    def _etapa(self, nome):
        """Registra a etapa da calibração (o visualizador 3D mostra esse estado)."""
        self.etapa_calibracao = nome
        self.ble.marcar(f"calibração: etapa '{nome}'")
        self._enviar_estado_visualizador(forcar=True)

    def _contagem(self, quadros, segundos, mensagem):
        """Mostra a instrução e espera alguns segundos (descartando os quadros)."""
        print(mensagem)
        fim = time.monotonic() + segundos
        impresso = None
        for _ in quadros:
            self._enviar_estado_visualizador()
            restante = int(math.ceil(fim - time.monotonic()))
            if restante <= 0:
                break
            if restante != impresso:
                print(f"  começando em {restante}...")
                impresso = restante

    def _relatar_instabilidade(self, motivo):
        """Diz ao usuário por que a janela atual não foi aceita (sem repetir a mesma frase)."""
        agora = time.monotonic()
        if motivo != self._calib_motivo or agora - self._calib_t_motivo > 5.0:
            print(f"  Aguardando estabilidade: {motivo}. Segure PARADA.")
            self._calib_motivo = motivo
            self._calib_t_motivo = agora

    def _coletar_parado(self, quadros, limite_dps, limite_direcao_graus):
        """Coleta amostras até existir uma janela realmente parada e de boa qualidade.
        Janela deslizante: um movimento só invalida as amostras afetadas. Retorna o dicionário
        de avaliar_janela_parada (aceleração média, offset do giro e métricas)."""
        janela = deque(maxlen=self.amostras_calibracao)
        inicio = time.monotonic()
        avisou_demora = False
        self._calib_motivo = None
        for quadro in quadros:
            if "erro" in quadro:
                self._aviso(f"Aviso do Arduino: {quadro['erro']} (verifique o GY-521).")
                continue
            dados = extrair_dados(quadro)
            if dados is None:
                continue
            janela.append((dados[0], dados[1]))
            self._enviar_estado_visualizador()
            if len(janela) < janela.maxlen:
                continue
            aprovada, motivo, resultado = avaliar_janela_parada(
                janela, limite_dps, limite_direcao_graus)
            if aprovada:
                print(f"  Etapa aprovada: {resultado['n']} amostras úteis "
                      f"({resultado['descartadas']} descartadas), giro pico-a-pico "
                      f"{resultado['pico_giro']:.2f} °/s, inclinação ±{resultado['espalhamento']:.2f}°.")
                return resultado
            self._relatar_instabilidade(motivo)
            if not avisou_demora and time.monotonic() - inicio > CALIB_AVISO_DEMORA_S:
                avisou_demora = True
                print(f"  Ainda sem uma leitura estável após {CALIB_AVISO_DEMORA_S:.0f} s. "
                      "Apoie melhor o dispositivo e verifique se o sensor está firme. "
                      "Se o problema persistir, os limites LIMITE_* da calibração podem estar "
                      "rígidos demais para o seu sensor. Continuo tentando (Ctrl+C sai).")
        raise ErroComunicacao("A comunicação terminou durante a calibração.")

    def _calibrar(self, quadros):
        self.calibrando = True
        self.ble.marcar("calibração iniciada")
        try:
            self._calibrar_etapas(quadros)
        finally:
            self.calibrando = False
            self.etapa_calibracao = ""
            self._enviar_estado_visualizador(forcar=True)

    def _calibrar_etapas(self, quadros):
        descobrir_cano = BARREL_AXIS is None
        validar_cano = descobrir_cano and VALIDAR_CANO_ABAIXANDO

        n = 0
        n_apoio = n_levantar = n_abaixar = None
        if CALIBRAR_OFFSET_APOIADO:
            n += 1
            n_apoio = n
        if descobrir_cano:
            n += 1
            n_levantar = n
        n += 1
        n_centro = n
        if validar_cano:
            n += 1
            n_abaixar = n
        total = n

        # Offset do giroscópio: com a arma apoiada o valor é bem preciso, e é ele
        # que determina o quanto a mira derrapa para os lados com o tempo.
        offset_apoiado = None
        if CALIBRAR_OFFSET_APOIADO:
            while True:
                self._etapa("apoiar")
                self._contagem(
                    quadros, 4,
                    f"\nPASSO {n_apoio}/{total}: APOIE a arma numa superfície firme (mesa) "
                    "e não encoste nela.")
                r = self._coletar_parado(quadros, LIMITE_APOIADO_DPS, LIMITE_DIRECAO_APOIADO_GRAUS)
                if max(abs(v) for v in r["giro"]) > OFFSET_GIRO_MAX_DPS:
                    print(f"  Offset do giroscópio suspeito "
                          f"({r['giro'][0]:.1f}, {r['giro'][1]:.1f}, {r['giro'][2]:.1f} °/s; o "
                          f"esperado é menor que {OFFSET_GIRO_MAX_DPS:.0f}). A arma estava mesmo "
                          "parada? Repetindo esta etapa...")
                    continue
                offset_apoiado = r["giro"]
                print(f"  Offset do giroscópio: {offset_apoiado[0]:.2f}, "
                      f"{offset_apoiado[1]:.2f}, {offset_apoiado[2]:.2f} °/s")
                break

        while True:
            giros_na_mao = []
            acel_levantada = acel_abaixada = None

            # Levantar a ponta: a mudança da direção "para cima" revela o eixo do cano.
            if descobrir_cano:
                self._etapa("levantar")
                self._contagem(
                    quadros, 4,
                    f"\nPASSO {n_levantar}/{total}: pegue a arma e LEVANTE a ponta uns 30 a 45 "
                    "graus acima do centro da tela, SEM torcer o pulso, e segure PARADA.")
                r = self._coletar_parado(quadros, LIMITE_NA_MAO_DPS, LIMITE_DIRECAO_NA_MAO_GRAUS)
                acel_levantada = r["acel"]
                giros_na_mao.append(r["giro"])

            # Pose final: apontando para o centro. É daqui que o cursor parte.
            self._etapa("centro")
            self._contagem(
                quadros, 4,
                f"\nPASSO {n_centro}/{total}: aponte a arma para o CENTRO da tela "
                "e segure PARADA.")
            r = self._coletar_parado(quadros, LIMITE_NA_MAO_DPS, LIMITE_DIRECAO_NA_MAO_GRAUS)
            acel_centro = r["acel"]
            giros_na_mao.append(r["giro"])

            # Conferência do eixo: abaixar a ponta deve dar o MESMO eixo que levantar.
            if validar_cano:
                self._etapa("abaixar")
                self._contagem(
                    quadros, 4,
                    f"\nPASSO {n_abaixar}/{total}: agora ABAIXE a ponta uns 30 a 45 graus "
                    "abaixo do centro, SEM torcer o pulso, e segure PARADA.")
                r = self._coletar_parado(quadros, LIMITE_NA_MAO_DPS, LIMITE_DIRECAO_NA_MAO_GRAUS)
                acel_abaixada = r["acel"]
                giros_na_mao.append(r["giro"])

            if descobrir_cano:
                eixo, angulo = calcular_eixo_do_cano(acel_centro, acel_levantada)
                if eixo is None or not (LEVANTE_MIN_GRAUS <= angulo <= LEVANTE_MAX_GRAUS):
                    print(f"  A ponta subiu {angulo:.0f}° (precisa ser de "
                          f"{LEVANTE_MIN_GRAUS:.0f} a {LEVANTE_MAX_GRAUS:.0f}°). "
                          "Etapa REPROVADA, refazendo as etapas do cano...")
                    continue
                print(f"  A ponta subiu {angulo:.0f}°. Eixo (subindo): "
                      f"({eixo[0]:+.3f}, {eixo[1]:+.3f}, {eixo[2]:+.3f})")
                if validar_cano:
                    eixo_baixo, angulo_baixo = calcular_eixo_do_cano(acel_centro, acel_abaixada)
                    if eixo_baixo is None or not (LEVANTE_MIN_GRAUS <= angulo_baixo <= LEVANTE_MAX_GRAUS):
                        print(f"  A ponta desceu {angulo_baixo:.0f}° (precisa ser de "
                              f"{LEVANTE_MIN_GRAUS:.0f} a {LEVANTE_MAX_GRAUS:.0f}°). "
                              "Etapa REPROVADA, refazendo as etapas do cano...")
                        continue
                    eixo_baixo = tuple(-v for v in eixo_baixo)  # descer inverte o sentido
                    diferenca = angulo_entre(eixo, eixo_baixo)
                    print(f"  A ponta desceu {angulo_baixo:.0f}°. Eixo (descendo): "
                          f"({eixo_baixo[0]:+.3f}, {eixo_baixo[1]:+.3f}, {eixo_baixo[2]:+.3f})")
                    if diferenca > TOLERANCIA_EIXO_CANO_GRAUS:
                        print(f"  Os dois eixos diferem {diferenca:.1f}° (máximo "
                              f"{TOLERANCIA_EIXO_CANO_GRAUS:.0f}°): o pulso torceu ou a arma "
                              "girou. Não dá para confiar no eixo. Etapa REPROVADA, refazendo...")
                        continue
                    print(f"  Eixo CONFERIDO: subir e descer concordam (diferença {diferenca:.1f}°).")
                    eixo = unitario(tuple(a + b for a, b in zip(eixo, eixo_baixo)))
                self.barril = eixo
                print(f"  Eixo do cano no sensor: ({eixo[0]:+.3f}, {eixo[1]:+.3f}, {eixo[2]:+.3f})")
                print(f"  Para pular o passo de levantar, use: "
                      f"BARREL_AXIS = ({eixo[0]:.3f}, {eixo[1]:.3f}, {eixo[2]:.3f})")
            else:
                self.barril = unitario(BARREL_AXIS)
            break

        # Offset final do giroscópio
        if offset_apoiado is not None:
            self.offset_giro = list(offset_apoiado)
        else:
            n_g = len(giros_na_mao)
            self.offset_giro = [sum(g[i] for g in giros_na_mao) / n_g for i in range(3)]

        # Inicializa a orientação deixando o acelerômetro assentar a inclinação
        self.q = (1.0, 0.0, 0.0, 0.0)
        for _ in range(150):
            self._filtro(acel_centro, (0.0, 0.0, 0.0), 0.01, GANHO_ACEL_INICIAL)

        self.acel_filtrada = None
        self.tempo_repouso = 0.0
        self.t_anterior = None
        self._definir_centro()
        self.ble.marcar("calibração concluída")
        self._primeiro_quadro_ativo = True
        print("Calibração concluída: a mira está em (0, 0), o centro da tela.\n")

    # ---------- botões ----------
    def _atualizar_botoes(self, b1, b2):
        """Detecta pressionar/soltar comparando com o estado anterior."""
        if b1 != self.esq_pressionado:
            self.ble.marcar(f"botão 1 {'PRESSIONADO' if b1 else 'solto'}")
            if self.debug:
                print(f">>> Botão 1 (esquerdo) {'PRESSIONADO' if b1 else 'SOLTO'}")
            if b1:
                # o "tranco" do gatilho não pode deslocar a mira: congela o cursor um instante
                self.trava_ate = time.monotonic() + TRAVA_GATILHO_MS / 1000.0
            if not self.monitor:
                if b1:
                    self.mouse.press(Button.left)
                else:
                    self.mouse.release(Button.left)
            self.esq_pressionado = b1
        if b2 != self.dir_pressionado:
            self.ble.marcar(f"botão 2 {'PRESSIONADO' if b2 else 'solto'}")
            if self.debug:
                print(f">>> Botão 2 (direito) {'PRESSIONADO' if b2 else 'SOLTO'}")
            if not self.monitor:
                if b2:
                    self.mouse.press(Button.right)
                else:
                    self.mouse.release(Button.right)
            self.dir_pressionado = b2

    def soltar_botoes(self):
        """Garante que nenhum botão fique 'preso' (desconexão, pausa ou saída)."""
        if self.monitor:
            return
        if self.esq_pressionado:
            self.mouse.release(Button.left)
            self.esq_pressionado = False
        if self.dir_pressionado:
            self.mouse.release(Button.right)
            self.dir_pressionado = False

    # ---------- movimento do cursor ----------
    def _mover_cursor(self, x, y):
        """Coloca o cursor na posição (coordenadas com centro em 0,0)."""
        px = int(round(self.largura / 2.0 + x))
        py = int(round(self.altura / 2.0 - y))  # tela: y cresce para baixo
        px = max(0, min(self.largura - 1, px))
        py = max(0, min(self.altura - 1, py))
        ultimo = self._ultimo_px
        # zona estável: ignora mudanças de poucos pixels (resto do tremor)
        if ultimo is None or abs(px - ultimo[0]) >= ZONA_ESTAVEL_PX or abs(py - ultimo[1]) >= ZONA_ESTAVEL_PX:
            self.mouse.position = (px, py)
            self._ultimo_px = (px, py)

    # ---------- envio para o visualizador 3D ----------
    def _enviar_visualizador(self, ang_x, ang_y, x, y, b1, b2):
        """Envia a orientação e a mira atuais ao visualizador 3D (se ele estiver aberto).
        Qualquer falha aqui é ignorada: o visualizador nunca pode atrapalhar o controlador."""
        self._ult_viz = (ang_x, ang_y, x, y, b1, b2)
        if self._udp is None:
            return
        self._ultimo_envio_viz = time.monotonic()
        pacote = {
            "v": 1,
            "q": [round(v, 5) for v in self.q],               # orientação (w, x, y, z)
            "barril": [round(v, 5) for v in self.barril],     # eixo do cano no sensor
            "ang": [round(ang_x, 3), round(ang_y, 3)],        # ângulos da mira (graus)
            "px": [round(x, 1), round(y, 1)],                 # cursor, centro da tela = (0,0)
            "b1": int(b1),
            "b2": int(b2),
            "ativo": bool(self.ativo),
            "trava": time.monotonic() < self.trava_ate,
            "tan": [round(self.tan_meia_h, 5), round(self.tan_meia_v, 5)],
            "tela": [self.largura, self.altura],
            "yaw0": round(self.yaw0, 5),
            # campos acrescentados (o visualizador pode ignorá-los):
            "conexao": self.estado_conexao,                   # procurando | conectado | desconectado
            "calibrando": bool(self.calibrando),
            "etapa": self.etapa_calibracao,                   # apoiar | levantar | centro | abaixar | ""
        }
        try:
            self._udp.sendto(json.dumps(pacote, separators=(",", ":")).encode("ascii"),
                             self._destino_3d)
        except (OSError, ValueError, TypeError):
            pass  # visualizador fechado ou rede local indisponível: ignora

    def _enviar_estado_visualizador(self, forcar=False):
        """Reenvia o último estado conhecido (conexão/calibração) quando não há quadros novos
        sendo processados, no máximo ~5 vezes por segundo."""
        if self._udp is None:
            return
        if not forcar and time.monotonic() - self._ultimo_envio_viz < 0.2:
            return
        self._enviar_visualizador(*self._ult_viz)

    # ---------- processamento de cada quadro ----------
    def _atualizar_orientacao(self, aceleracao, giro):
        agora = time.perf_counter()
        if self.t_anterior is None:
            self.t_anterior = agora
            return
        dt = min(max(agora - self.t_anterior, 0.001), self.dt_max)
        self.t_anterior = agora
        self.dt = dt

        # Corrige o offset do giroscópio quando o dispositivo está parado
        desvio = max(abs(giro[i] - self.offset_giro[i]) for i in range(3))
        if desvio < REPOUSO_GIRO_DPS and abs(norma(aceleracao) - 1.0) < REPOUSO_ACEL_G:
            self.tempo_repouso += dt
        else:
            self.tempo_repouso = 0.0
        if self.tempo_repouso > REPOUSO_TEMPO_S:
            for i in range(3):
                self.offset_giro[i] += ADAPTACAO_OFFSET * (giro[i] - self.offset_giro[i])

        giro_dps = tuple(giro[i] - self.offset_giro[i] for i in range(3))
        giro_rad = tuple(math.radians(v) for v in giro_dps)

        # O acelerômetro na ponta também sente a aceleração do giro da arma. Por isso:
        # 1) passa por um filtro passa-baixa (tira o tremor de ~10 Hz);
        # 2) perde peso quanto mais rápido a arma gira (nesse caso confia-se no giroscópio).
        if self.acel_filtrada is None:
            self.acel_filtrada = list(aceleracao)
        else:
            alfa = dt / (ACEL_TAU_S + dt)
            for i in range(3):
                self.acel_filtrada[i] += alfa * (aceleracao[i] - self.acel_filtrada[i])
        peso = max(0.0, 1.0 - norma(giro_dps) / GIRO_MAX_CORRECAO_DPS)
        self._filtro(self.acel_filtrada, giro_rad, dt, GANHO_ACEL * peso)

    def _processar(self, quadro):
        if "erro" in quadro:
            self._aviso(f"Aviso do Arduino: {quadro['erro']} (verifique o GY-521).")
            return
        dados = extrair_dados(quadro)
        if dados is None:
            return  # quadro de status ou incompleto
        self._ultimo_aviso = None
        aceleracao, giro, b1, b2 = dados

        if self._primeiro_quadro_ativo:
            self._primeiro_quadro_ativo = False
            self.ble.marcar("primeiro quadro processado após a calibração")

        self._atualizar_orientacao(aceleracao, giro)

        if self.pedido_recentro:
            self.pedido_recentro = False
            self._definir_centro()
            print("Recentralizado: a ponta atual agora é o centro (0, 0).")

        # ângulos -> filtro anti-tremor -> pixels
        ang_x, ang_y = self._angulos_relativos()
        ang_x_f = self.filtro_x(ang_x, self.dt)
        ang_y_f = self.filtro_y(ang_y, self.dt)
        x, y = self._para_pixels(ang_x_f, ang_y_f)
        self._enviar_visualizador(ang_x_f, ang_y_f, x, y, b1, b2)

        if self.monitor:
            self._atualizar_botoes(b1, b2)  # só registra e mostra os eventos
        elif self.ativo:
            self._atualizar_botoes(b1, b2)
            if time.monotonic() >= self.trava_ate:
                self._mover_cursor(x, y)
        else:
            self.soltar_botoes()

        # Mostra os dados ~5 vezes por segundo
        if self.debug and time.monotonic() - self._ultimo_debug > 0.2:
            self._ultimo_debug = time.monotonic()
            print(f"[dados] acel(g): {aceleracao[0]:6.2f} {aceleracao[1]:6.2f} {aceleracao[2]:6.2f} | "
                  f"giro(°/s): {giro[0]:7.1f} {giro[1]:7.1f} {giro[2]:7.1f} | "
                  f"mira: x={x:+6.0f} y={y:+6.0f} px (ang h={ang_x:+6.1f}° v={ang_y:+6.1f}°) | "
                  f"b1={int(b1)} b2={int(b2)} | {self.ble.resumo()}")

    # ---------- sessão com uma conexão ----------
    def _mensagem_ativa(self):
        print("Arma ativa! F7 = recalibrar | F8 = recentralizar | F9 = pausar/retomar | "
              "Ctrl+C = sair.")

    def _sessao(self):
        quadros = self._ler_quadros()
        if self.monitor:
            print("MODO MONITOR: o cursor NÃO será movido.")
        self.pedido_recalibrar = False
        self._calibrar(quadros)
        self._mensagem_ativa()
        for quadro in quadros:
            if self.pedido_recalibrar:
                # F7: refaz a calibração nesta mesma conexão, sem reiniciar o programa
                self.pedido_recalibrar = False
                self.soltar_botoes()
                print("\nRecalibrando...")
                self._calibrar(quadros)
                self.pedido_recentro = False
                self._mensagem_ativa()
                continue  # o quadro em mãos é anterior à calibração: descarta
            self._processar(quadro)

    # ---------- laço principal com reconexão ----------
    def executar(self):
        try:
            versao_bleak = importlib.metadata.version("bleak")
        except Exception:
            versao_bleak = "?"
        print(f"[diag] Python {platform.python_version()} | Bleak {versao_bleak} | "
              f"{platform.platform()}")
        self._iniciar_teclas()
        self.ble.iniciar()
        try:
            while True:
                self._aguardar_conexao()
                try:
                    self._sessao()
                except (ErroComunicacao, OSError) as erro:
                    self.soltar_botoes()
                    self.t_anterior = None
                    self.estado_conexao = "desconectado"
                    self._enviar_estado_visualizador(forcar=True)
                    if self.ble.conectado.is_set():
                        # a conexão continua aberta, mas parou de servir: derruba e reconecta
                        self.ble.forcar_reconexao()
                    self._aviso(f"ERRO de comunicação BLE: {erro}\n"
                                "  - Verifique se o HC-06 está ligado e dentro do alcance, e se "
                                "outro programa/aparelho não está conectado a ele.\n"
                                f"  Tentando reconectar a cada {INTERVALO_RECONEXAO_S:.0f} s...")
        except KeyboardInterrupt:
            print("\nEncerrando.")
        finally:
            self.soltar_botoes()
            self._parar_teclas()
            self.ble.parar()


def main():
    parser = argparse.ArgumentParser(description="Arma com Arduino + GY-521 (mira absoluta)")
    parser.add_argument("--address", metavar="ENDERECO",
                        help="endereço BLE do HC-06, ex.: EE:52:CB:4B:54:7C "
                             "(padrão: %s)" % ENDERECO_BLE)
    parser.add_argument("--list", action="store_true",
                        help="lista os dispositivos BLE próximos e sai")
    parser.add_argument("--fov", type=float,
                        help="campo de visão horizontal em graus; maior = menos sensível "
                             "(padrão: %.0f)" % FOV_HORIZONTAL_GRAUS)
    parser.add_argument("--cutoff", type=float,
                        help="firmeza anti-tremor em Hz; menor = mais firme "
                             "(padrão: %.1f)" % FILTRO_CUTOFF_HZ)
    parser.add_argument("--no-3d", action="store_true",
                        help="não envia dados ao visualizador 3D")
    parser.add_argument("--monitor", action="store_true",
                        help="mostra dados e mira, sem mover o cursor nem clicar")
    parser.add_argument("--debug", action="store_true",
                        help="mostra os dados e continua controlando o mouse")
    args = parser.parse_args()

    if args.list:
        listar_dispositivos_ble()
        return 0

    ArmaArduino(args).executar()
    return 0


if __name__ == "__main__":
    sys.exit(main())