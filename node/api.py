from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List

# FastAPI instance
app = FastAPI()

# Model for position data
class PositionData(BaseModel):
    robot_id: int
    x: float
    z: float
    theta: float
    status: str

# In-memory storage for position data
position_data: List[PositionData] = []

@app.post("/position")
def receive_position(data: PositionData):
    position_data.append(data)
    return {"message": "Data received successfully", "data": data}

@app.get("/position")
def get_positions():
    return {"positions": position_data}
