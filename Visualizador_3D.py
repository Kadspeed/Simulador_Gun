"""
Visualizador 3D da arma

Mostra um modelo 3D (STL ou um modelo simples parecido com a sua arma) girando em
tempo real do mesmo jeito que o dispositivo real. Ele NÃO lê a porta serial: os dados
vêm do arm.py (mouse_controller.py), que envia a orientação por UDP para este
computador. Assim os dois rodam ao mesmo tempo, sem brigar pela COM4, e o que você
vê aqui é exatamente o que o controle do mouse está usando.

O que aparece na tela:
  - o modelo da arma, girando com o dispositivo (o pivô é o ponto do SENSOR);
  - os eixos do sensor: X vermelho, Y verde, Z azul;
  - uma linha laranja: a direção do cano que o arm.py está usando;
  - uma "tela" virtual com um ponto ciano: onde o cursor do mouse estaria;
  - na esquerda, os números: ângulos, posição do cursor e botões.

Marcar pontos no modelo (para dizer onde ficam o sensor e o cano):
  1. aperte 1, 2 ou 3 (escolhe o que marcar: SENSOR, PONTA ou BASE do cano);
     o modelo congela;
  2. clique com o botão DIREITO do mouse sobre o ponto desejado do modelo
     (um clique rápido, sem arrastar);
  3. o modelo volta a se mexer. Aperte 5 para salvar.
  A direção do cano no modelo é BASE -> PONTA (os dois sobre o eixo do cano).
  Sem a BASE, usa-se SENSOR -> PONTA, que fica torta se o sensor estiver fora do eixo.

Teclas:
  1 / 2 / 3   escolher o ponto a marcar (sensor / ponta / base)
  botão direito  marcar o ponto escolhido sobre o modelo
  <- / ->     girar o modelo em torno do cano (acerta de que lado ficam bateria e fios)
  4           congelar / liberar o movimento
  5           salvar marcações e ajuste em visualizador_config.json
  mouse       girar a câmera (botão esquerdo arrastando), mover (botão do meio), zoom (roda)
  Q           sair

Uso:
  python visualizador_3d.py                     -> modelo simples parecido com a foto
  python visualizador_3d.py --stl minha_arma.stl
  python visualizador_3d.py --porta 49555       -> porta UDP (igual à do arm.py)
"""

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np
import pyvista as pv

# =====================================================================
# CONFIGURAÇÕES
# =====================================================================
PORTA_UDP = 49555                    # igual a PORTA_VISUALIZADOR no arm.py
DISTANCIA_TELA = 3.0                 # distância da tela virtual (o modelo tem ~1 de comprimento)
PASSO_ROLAGEM_GRAUS = 5.0
TEMPO_SEM_DADOS_S = 1.0
ARQUIVO_CONFIG = Path(__file__).with_name("visualizador_config.json")

COR_SENSOR = "red"
COR_PONTA = "yellow"
COR_BASE = "magenta"


# =====================================================================
# MATEMÁTICA
# =====================================================================
def unitario(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def matriz_do_quaternion(q):
    """Quaternion (w, x, y, z) -> matriz de rotação 3x3 (sensor -> mundo, z para cima)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rotacao_eixo(eixo, angulo):
    """Rotação de 'angulo' radianos em torno de 'eixo' (fórmula de Rodrigues)."""
    k = unitario(eixo)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angulo) * kx + (1 - math.cos(angulo)) * (kx @ kx)


def rotacao_entre(a, b):
    """Menor rotação que leva o vetor unitário a no vetor unitário b."""
    a, b = unitario(a), unitario(b)
    c = float(np.dot(a, b))
    if c > 1 - 1e-9:
        return np.eye(3)
    if c < -1 + 1e-9:  # opostos: gira 180° em torno de qualquer eixo perpendicular
        eixo = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(eixo) < 1e-6:
            eixo = np.cross(a, [0.0, 1.0, 0.0])
        return rotacao_eixo(eixo, math.pi)
    v = np.cross(a, b)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + (vx @ vx) / (1 + c)


def rotacao_z(angulo):
    c, s = math.cos(angulo), math.sin(angulo)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def matriz4(rotacao, translacao=(0.0, 0.0, 0.0)):
    m = np.eye(4)
    m[:3, :3] = rotacao
    m[:3, 3] = translacao
    return m


# =====================================================================
# MODELO
# =====================================================================
def _parte(malha, cor):
    """Pinta uma peça com uma cor RGB (para juntar várias peças em uma malha só)."""
    malha = malha.triangulate()
    malha.point_data["cores"] = np.tile(np.array(cor, dtype=np.uint8), (malha.n_points, 1))
    return malha


def modelo_simples():
    """Modelo aproximado a partir da foto: cano roxo na frente (+X), corpo verde,
    bateria 9 V, placa azul e lata preta inclinada como empunhadura.
    Retorna (malha, ponto_sensor, ponto_ponta, ponto_base)."""
    a15 = math.radians(15.0)
    partes = [
        _parte(pv.Cylinder(center=(0.48, 0, 0), direction=(1, 0, 0), radius=0.06, height=0.14, resolution=40), (130, 80, 170)),
        _parte(pv.Cylinder(center=(0.37, 0, 0), direction=(1, 0, 0), radius=0.085, height=0.08, resolution=40), (110, 60, 150)),
        _parte(pv.Cylinder(center=(0.065, 0, 0), direction=(1, 0, 0), radius=0.075, height=0.53, resolution=40), (130, 210, 185)),
        _parte(pv.Cylinder(center=(-0.20 - 0.2 * math.cos(a15), 0, -0.2 * math.sin(a15)),
                           direction=(-math.cos(a15), 0, -math.sin(a15)), radius=0.08, height=0.4, resolution=40), (35, 30, 35)),
        _parte(pv.Cube(center=(0.18, 0, 0.098), x_length=0.13, y_length=0.07, z_length=0.045), (200, 40, 40)),
        _parte(pv.Cube(center=(-0.02, 0, 0.125), x_length=0.06, y_length=0.045, z_length=0.012), (40, 90, 220)),
        _parte(pv.Cube(center=(-0.14, 0, 0.09), x_length=0.2, y_length=0.1, z_length=0.02), (60, 60, 70)),
    ]
    combinado = pv.merge(partes, merge_points=False)
    if not isinstance(combinado, pv.PolyData):
        combinado = combinado.extract_surface()
    sensor = np.array([-0.02, 0.0, 0.125])
    ponta = np.array([0.55, 0.0, 0.0])
    base = np.array([-0.20, 0.0, 0.0])
    return combinado, sensor, ponta, base


def modelo_stl(caminho):
    """Carrega um STL, centraliza e escala para que a maior dimensão valha 1."""
    malha = pv.read(caminho)
    if not isinstance(malha, pv.PolyData):
        malha = malha.extract_surface()
    limites = malha.bounds
    extensao = max(limites[1] - limites[0], limites[3] - limites[2], limites[5] - limites[4])
    centro = np.array([(limites[0] + limites[1]) / 2, (limites[2] + limites[3]) / 2,
                       (limites[4] + limites[5]) / 2])
    malha = malha.copy()
    malha.points = (np.asarray(malha.points) - centro) / extensao
    return malha


# =====================================================================
# APLICATIVO
# =====================================================================
class Visualizador:
    def __init__(self, args):
        self.args = args
        self.stl = str(Path(args.stl).resolve()) if args.stl else None

        # Modelo e pontos padrão
        if self.stl:
            self.malha = modelo_stl(self.stl)
            self.p_sensor = np.zeros(3)
            self.p_ponta = None
            self.p_base = None
            self.cor_modelo = None
        else:
            self.malha, self.p_sensor, self.p_ponta, self.p_base = modelo_simples()
            self.cor_modelo = "cores"
        self.rolagem_graus = 0.0

        self._carregar_config()

        # Rede (UDP local, sem bloquear)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", args.porta))
        self.sock.setblocking(False)

        # Estado
        self.pacote = None
        self.t_ultimo_pacote = 0.0
        self.contagem = 0
        self.taxa = 0
        self.t_taxa = time.monotonic()
        self.congelado = False
        self.modo_marca = None            # "sensor", "ponta" ou "base"
        self.M_modelo = np.eye(4)
        self.M_sensor = np.eye(4)
        self.atores_marca = {}
        self.tela_chave = None
        self.ator_tela = None
        self.ator_contorno = None

        self._montar_cena()

    # ---------- configuração salva ----------
    def _carregar_config(self):
        if not ARQUIVO_CONFIG.exists():
            return
        try:
            cfg = json.loads(ARQUIVO_CONFIG.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return
        if cfg.get("stl") != self.stl:  # marcações pertencem a outro modelo
            return
        for chave, atributo in (("p_sensor", "p_sensor"), ("p_ponta", "p_ponta"), ("p_base", "p_base")):
            valor = cfg.get(chave)
            if isinstance(valor, list) and len(valor) == 3:
                setattr(self, atributo, np.array(valor, dtype=float))
        self.rolagem_graus = float(cfg.get("rolagem_graus", 0.0))
        print(f"Marcações carregadas de {ARQUIVO_CONFIG.name}.")

    def _salvar_config(self):
        cfg = {
            "stl": self.stl,
            "p_sensor": None if self.p_sensor is None else [float(v) for v in self.p_sensor],
            "p_ponta": None if self.p_ponta is None else [float(v) for v in self.p_ponta],
            "p_base": None if self.p_base is None else [float(v) for v in self.p_base],
            "rolagem_graus": float(self.rolagem_graus),
        }
        ARQUIVO_CONFIG.write_text(json.dumps(cfg, indent=2), encoding="utf-8")
        print(f"Salvo em {ARQUIVO_CONFIG}")

    # ---------- direção do cano no modelo ----------
    def direcao_modelo(self):
        """Direção do cano no sistema do modelo: BASE -> PONTA (ou SENSOR -> PONTA)."""
        if self.p_ponta is None:
            return np.array([1.0, 0.0, 0.0])
        origem = self.p_base if self.p_base is not None else self.p_sensor
        d = np.asarray(self.p_ponta) - np.asarray(origem)
        return unitario(d) if np.linalg.norm(d) > 1e-6 else np.array([1.0, 0.0, 0.0])

    # ---------- cena ----------
    def _montar_cena(self):
        self.plotter = pv.Plotter(window_size=(1200, 800), title="Visualizador 3D da arma")
        self.plotter.set_background("#1b1f27")

        if self.cor_modelo:
            self.ator_modelo = self.plotter.add_mesh(self.malha, scalars=self.cor_modelo, rgb=True,
                                                     smooth_shading=False, name="modelo")
        else:
            self.ator_modelo = self.plotter.add_mesh(self.malha, color="#9fb4c7",
                                                     smooth_shading=True, name="modelo")

        # eixos do sensor (X vermelho, Y verde, Z azul)
        self.atores_eixos = []
        for direcao, cor in (((1, 0, 0), "red"), ((0, 1, 0), "lime"), ((0, 0, 1), "deepskyblue")):
            seta = pv.Arrow(start=(0, 0, 0), direction=direcao, scale=0.4)
            self.atores_eixos.append(self.plotter.add_mesh(seta, color=cor, pickable=False))

        # linha laranja: direção do cano usada pelo arm.py
        self.linha_cano = pv.Line((0, 0, 0), (1, 0, 0))
        self.plotter.add_mesh(self.linha_cano, color="orange", line_width=4, pickable=False)

        # ponto ciano: onde o cursor estaria na tela virtual
        self.ator_cursor = self.plotter.add_mesh(pv.Sphere(radius=0.06), color="cyan", pickable=False)

        # chão de referência
        chao = pv.Plane(center=(0.5, 0, -0.8), direction=(0, 0, 1), i_size=6, j_size=6,
                        i_resolution=12, j_resolution=12)
        self.plotter.add_mesh(chao, style="wireframe", color="#444b57", opacity=0.6, pickable=False)

        self.texto = self.plotter.add_text("Aguardando o arm.py...", position=(10, 10),
                                           font_size=10, color="white", name="hud")
        if hasattr(self.texto, "GetTextProperty"):  # ancora o texto no canto superior esquerdo
            self.texto.GetTextProperty().SetVerticalJustificationToTop()
            self.texto.GetPositionCoordinate().SetCoordinateSystemToNormalizedViewport()
            self.texto.SetPosition(0.01, 0.99)
        self.plotter.camera_position = [(-4.5, -3.6, 2.2), (1.4, 0.0, 0.0), (0, 0, 1)]

        # marcações já salvas
        for nome in ("sensor", "ponta", "base"):
            self._criar_marca(nome)

        # teclas
        self.plotter.add_key_event("1", lambda: self._escolher_marca("sensor"))
        self.plotter.add_key_event("2", lambda: self._escolher_marca("ponta"))
        self.plotter.add_key_event("3", lambda: self._escolher_marca("base"))
        self.plotter.add_key_event("4", self._alternar_congelar)
        self.plotter.add_key_event("5", self._salvar_config)
        self.plotter.add_key_event("Left", lambda: self._rolar(-PASSO_ROLAGEM_GRAUS))
        self.plotter.add_key_event("Right", lambda: self._rolar(PASSO_ROLAGEM_GRAUS))
        try:
            self.plotter.enable_point_picking(callback=self._ao_marcar, picker="cell",
                                              show_message=False, show_point=False)
        except TypeError:  # versões antigas do PyVista
            self.plotter.enable_point_picking(callback=self._ao_marcar,
                                              show_message=False, show_point=False)

    # ---------- marcações ----------
    def _ponto(self, nome):
        return {"sensor": self.p_sensor, "ponta": self.p_ponta, "base": self.p_base}[nome]

    def _criar_marca(self, nome):
        ponto = self._ponto(nome)
        if ponto is None:
            return
        cor = {"sensor": COR_SENSOR, "ponta": COR_PONTA, "base": COR_BASE}[nome]
        ator = self.plotter.add_mesh(pv.Sphere(radius=0.022, center=ponto), color=cor,
                                     name=f"marca_{nome}", pickable=False)
        ator.user_matrix = self.M_modelo
        self.atores_marca[nome] = ator

    def _escolher_marca(self, nome):
        self.modo_marca = nome
        self.congelado = True
        print(f"Marcar {nome.upper()}: clique com o botão DIREITO sobre o ponto no modelo.")

    def _para_modelo(self, ponto_mundo):
        """Converte um ponto do mundo (onde o clique caiu) para as coordenadas do modelo."""
        M = self.M_modelo
        return M[:3, :3].T @ (np.asarray(ponto_mundo, dtype=float) - M[:3, 3])

    def _ao_marcar(self, ponto_mundo, *_):
        if self.modo_marca is None:
            print("Antes escolha o que marcar: 1 = sensor, 2 = ponta, 3 = base do cano.")
            return
        if ponto_mundo is None or len(ponto_mundo) != 3:
            return
        ponto = self._para_modelo(ponto_mundo)  # o modelo está congelado durante a marcação
        nome = self.modo_marca
        if nome == "sensor":
            self.p_sensor = ponto
        elif nome == "ponta":
            self.p_ponta = ponto
        else:
            self.p_base = ponto
        print(f"{nome.upper()} marcado em ({ponto[0]:+.3f}, {ponto[1]:+.3f}, {ponto[2]:+.3f}).")
        self._criar_marca(nome)
        self.modo_marca = None
        self.congelado = False

    def _alternar_congelar(self):
        self.congelado = not self.congelado
        print("Movimento CONGELADO." if self.congelado else "Movimento liberado.")

    def _rolar(self, graus):
        self.rolagem_graus = (self.rolagem_graus + graus) % 360.0

    # ---------- rede ----------
    def _receber(self):
        while True:
            try:
                dados, _ = self.sock.recvfrom(4096)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            try:
                pacote = json.loads(dados.decode("ascii"))
            except ValueError:
                continue
            if isinstance(pacote, dict) and "q" in pacote and "barril" in pacote:
                self.pacote = pacote
                self.t_ultimo_pacote = time.monotonic()
                self.contagem += 1
        agora = time.monotonic()
        if agora - self.t_taxa >= 1.0:
            self.taxa = self.contagem
            self.contagem = 0
            self.t_taxa = agora

    # ---------- atualização da cena ----------
    def _atualizar_transformacoes(self, pacote):
        q = pacote["q"]
        barril = unitario(pacote["barril"])
        R_sensor = matriz_do_quaternion(q)

        # modelo -> sensor: leva a direção do cano do modelo ao eixo do cano do sensor,
        # depois aplica a rolagem escolhida em torno do cano do modelo
        d_modelo = self.direcao_modelo()
        A = rotacao_entre(d_modelo, barril) @ rotacao_eixo(d_modelo, math.radians(self.rolagem_graus))
        Rm = R_sensor @ A
        self.M_modelo = matriz4(Rm, -Rm @ np.asarray(self.p_sensor))  # o pivô é o ponto do sensor
        self.M_sensor = matriz4(R_sensor)

        self.ator_modelo.user_matrix = self.M_modelo
        for ator in self.atores_marca.values():
            ator.user_matrix = self.M_modelo
        for ator in self.atores_eixos:
            ator.user_matrix = self.M_sensor

        direcao_mundo = R_sensor @ barril
        self.linha_cano.points = np.array([[0.0, 0.0, 0.0], direcao_mundo * DISTANCIA_TELA * 1.35])

    def _atualizar_tela(self, pacote):
        tan_h, tan_v = pacote["tan"]
        largura, altura = pacote["tela"]
        yaw0 = float(pacote["yaw0"])
        chave = (round(tan_h, 4), largura, altura)
        if chave != self.tela_chave:
            self.tela_chave = chave
            w = 2 * DISTANCIA_TELA * tan_h
            h = 2 * DISTANCIA_TELA * tan_v
            d = DISTANCIA_TELA
            pontos = np.array([[d, -w / 2, -h / 2], [d, w / 2, -h / 2], [d, w / 2, h / 2], [d, -w / 2, h / 2]])
            plano = pv.PolyData(pontos, faces=np.array([4, 0, 1, 2, 3]))
            contorno = pv.lines_from_points(np.vstack([pontos, pontos[0]]))
            self.ator_tela = self.plotter.add_mesh(plano, color="#3b82f6", opacity=0.18,
                                                   name="tela", pickable=False)
            self.ator_contorno = self.plotter.add_mesh(contorno, color="#60a5fa", line_width=3,
                                                       name="tela_contorno", pickable=False)
        Rz = matriz4(rotacao_z(yaw0))
        self.ator_tela.user_matrix = Rz
        self.ator_contorno.user_matrix = Rz

        # posição do cursor na tela virtual (x para a direita = -Y antes de girar pelo yaw)
        x_px, y_px = pacote["px"]
        local = np.array([DISTANCIA_TELA,
                          -DISTANCIA_TELA * tan_h * x_px / (largura / 2.0),
                          DISTANCIA_TELA * tan_v * y_px / (altura / 2.0)])
        self.ator_cursor.position = tuple(rotacao_z(yaw0) @ local)

    @staticmethod
    def _definir_texto(ator, texto):
        if hasattr(ator, "SetInput"):
            ator.SetInput(texto)
        else:
            ator.SetText(0, texto)

    def _atualizar_texto(self):
        agora = time.monotonic()
        if self.pacote is None:
            estado = "Aguardando o arm.py... (ele está rodando e já terminou a calibração?)"
        elif agora - self.t_ultimo_pacote > TEMPO_SEM_DADOS_S:
            estado = "SEM DADOS do arm.py (parou ou foi fechado)"
        else:
            estado = f"Conectado: {self.taxa} pacotes/s"
        linhas = ["ARMA 3D - " + estado]
        if self.pacote is not None:
            p = self.pacote
            linhas.append(f"Mira: x={p['px'][0]:+6.0f} y={p['px'][1]:+6.0f} px   "
                          f"ang h={p['ang'][0]:+6.1f}  v={p['ang'][1]:+6.1f} graus")
            linhas.append(f"Botão 1: {'APERTADO' if p['b1'] else 'solto'}   "
                          f"Botão 2: {'APERTADO' if p['b2'] else 'solto'}   "
                          f"Controle: {'ATIVO' if p.get('ativo', True) else 'PAUSADO'}"
                          f"{'  (gatilho: cursor travado)' if p.get('trava') else ''}")
            b = p["barril"]
            linhas.append(f"Eixo do cano no sensor: ({b[0]:+.2f}, {b[1]:+.2f}, {b[2]:+.2f})")
        linhas.append(f"Rolagem do modelo: {self.rolagem_graus:.0f} graus   "
                      f"{'[CONGELADO]' if self.congelado else ''}")
        if self.modo_marca:
            linhas.append(f">> Marcando {self.modo_marca.upper()}: clique com o botão DIREITO sobre o ponto do modelo")
        linhas.append("")
        linhas.append("1 sensor  2 ponta  3 base do cano  (depois botão direito no modelo)   "
                      "setas esq/dir: rolar   4: congelar   5: salvar")
        linhas.append("Eixos do sensor: X vermelho, Y verde, Z azul   linha laranja = cano   ponto ciano = cursor")
        if self.p_ponta is None:
            linhas.append("Dica: marque a PONTA (tecla 2) e a BASE do cano (tecla 3) no modelo.")
        self._definir_texto(self.texto, "\n".join(linhas))

    # ---------- laço principal ----------
    def executar(self):
        print(__doc__.split("Teclas:")[0].strip().splitlines()[0])
        print(f"Escutando o arm.py na porta UDP {self.args.porta}. Feche a janela ou aperte Q para sair.")
        self.plotter.show(interactive_update=True, auto_close=False)
        try:
            while not getattr(self.plotter, "_closed", False):
                self._receber()
                if self.pacote is not None and not self.congelado:
                    self._atualizar_transformacoes(self.pacote)
                    self._atualizar_tela(self.pacote)
                elif self.pacote is not None:
                    # congelado: o modelo fica parado, mas as marcações já usam M_modelo
                    pass
                self._atualizar_texto()
                self.plotter.update()
                time.sleep(0.008)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                self.plotter.close()
            except Exception:
                pass
            self.sock.close()


def main():
    parser = argparse.ArgumentParser(description="Visualizador 3D da arma (recebe dados do arm.py)")
    parser.add_argument("--stl", help="arquivo STL do dispositivo (padrão: modelo simples)")
    parser.add_argument("--porta", type=int, default=PORTA_UDP, help="porta UDP (padrão: %d)" % PORTA_UDP)
    args = parser.parse_args()
    if args.stl and not Path(args.stl).exists():
        print(f"Arquivo não encontrado: {args.stl}")
        return 1
    Visualizador(args).executar()
    return 0


if __name__ == "__main__":
    sys.exit(main())