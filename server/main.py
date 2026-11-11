"""
main.py - starts the FastAPI application.

Run from the project root:
    uvicorn server.main:app --host 127.0.0.1 --port 8000
or:
    python -m server.main

Module map:
    config.py         paths, settings, users.json / robots.json
    auth.py           login, passwords, session tokens
    robot_manager.py  robot / browser / phone connections and availability
    routes.py         HTTP endpoints and the static client
    websocket.py      /ws authentication and message routing
"""

from fastapi import FastAPI

from .routes import router as http_router
from .websocket import router as websocket_router

app = FastAPI()

app.include_router(http_router)
app.include_router(websocket_router)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("server.main:app", host="127.0.0.1", port=8000)
