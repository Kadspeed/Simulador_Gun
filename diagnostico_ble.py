import asyncio
import time
from bleak import BleakClient, BleakScanner

ENDERECO = "EE:52:CB:4B:54:7C"
FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
LIMITE_S = 60

NOTIFICAR = True     # False: só conecta e fica parado (sem start_notify)
USAR_CACHE = True    # False: força o Windows a redescobrir os serviços
PAREAR = False       # True: chama pair() logo após conectar


async def main():
    print(f"Opções: NOTIFICAR={NOTIFICAR} USAR_CACHE={USAR_CACHE} PAREAR={PAREAR}")
    dispositivo = await BleakScanner.find_device_by_address(ENDERECO, timeout=10)
    if dispositivo is None:
        print("HC-06 não encontrado.")
        return
    caiu = asyncio.Event()
    queda = []
    contagem = [0]

    def ao_cair(_cliente):
        queda.append(time.monotonic())
        caiu.set()

    def ao_notificar(_c, _dados):
        contagem[0] += 1

    cliente = BleakClient(dispositivo, disconnected_callback=ao_cair,
                          winrt=dict(use_cached_services=USAR_CACHE))
    await cliente.connect()
    t0 = time.monotonic()
    try:
        print(f"Conectado. MTU: {cliente.mtu_size}")
        if PAREAR:
            try:
                print(f"pair() retornou: {await cliente.pair()}")
            except Exception as erro:
                print(f"pair() falhou: {erro!r}")
        if NOTIFICAR:
            await cliente.start_notify(FFE1, ao_notificar)
        print(f"Arma PARADA na mesa. Aguardando até {LIMITE_S} s...")
        proximo = 10
        while not caiu.is_set() and time.monotonic() - t0 < LIMITE_S:
            await asyncio.sleep(0.5)
            if time.monotonic() - t0 >= proximo:
                print(f"  {proximo} s, {contagem[0]} notificações")
                proximo += 10
    finally:
        try:
            await cliente.disconnect()
        except Exception:
            pass
    if queda:
        print(f"CAIU após {queda[0] - t0:.1f} s ({contagem[0]} notificações).")
    else:
        print(f"Não caiu em {LIMITE_S} s ({contagem[0]} notificações).")


asyncio.run(main())