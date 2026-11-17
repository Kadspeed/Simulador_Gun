
import asyncio
from bleak import BleakClient

ADDRESS = "EE:52:CB:4B:54:7C"

async def main():
    async with BleakClient(ADDRESS) as client:
        print("Conectado:", client.is_connected)

        services = await client.get_services()

        for service in services:
            print("\nServiço:", service.uuid)

            for char in service.characteristics:
                print("  Característica:", char.uuid)
                print("  Propriedades:", char.properties)

asyncio.run(main())
