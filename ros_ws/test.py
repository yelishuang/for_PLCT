import asyncio
import websockets

clients = set()

async def handler(websocket):
    print("新客户端连接")
    clients.add(websocket)
    try:
        async for message in websocket:
            print(f"收到消息: {message}")
            # 广播消息并自动清理无效连接
            dead_clients = []
            for client in clients:
                if client != websocket:
                    try:
                        await client.send(message)
                    except websockets.exceptions.ConnectionClosed:
                        dead_clients.append(client)
            for dead_client in dead_clients:
                clients.remove(dead_client)
    except websockets.exceptions.ConnectionClosed:
        print("客户端断开连接")
    finally:
        clients.discard(websocket)

async def main():
    async with websockets.serve(handler, "0.0.0.0", 8765):
        print("WebSocket 服务器运行在 ws://localhost:8765")
        await asyncio.Future()

asyncio.run(main())
