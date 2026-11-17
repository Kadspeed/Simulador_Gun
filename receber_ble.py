
import asyncio
from bleak import BleakClient

MAC = "EE:52:CB:4B:54:7C"
CARACTERISTICA = "0000ffe1-0000-1000-8000-00805f9b34fb"

def receber(sender, data):
    print("RECEBIDO:", data.decode("utf-8", errors="replace").strip())

async def main():
    print("Conectando...")

    try:
        async with BleakClient(MAC, timeout=15) as client:
            print("CONECTADO!")

            await client.start_notify(CARACTERISTICA, receber)

            print("Aguardando dados por 20 segundos...")

            await asyncio.sleep(20)

            await client.stop_notify(CARACTERISTICA)

            print("Teste finalizado.")

    except Exception as erro:
        print("ERRO:", repr(erro))

asyncio.run(main())
