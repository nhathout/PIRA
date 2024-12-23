import streamlit as st
import requests
import pandas as pd
import plotly.graph_objects as go
import time

# Define API endpoint
API_ENDPOINT = "http://127.0.0.1:8000/position"

# Refresh interval (in seconds)
REFRESH_INTERVAL = 0.5  # Adjust as needed

# Track bounds
TRACK_BOUNDS = {
    "bottom_left": (-1159, -754),
    "bottom_right": (1204, -872),
    "top_right": (1168, 1119),
    "top_left": (-1173, 1171),
}

# Fetch data from the API
def fetch_positions():
    try:
        response = requests.get(API_ENDPOINT)
        if response.status_code == 200:
            return response.json().get("positions", [])
        else:
            st.error(f"Failed to fetch data from API. Status code: {response.status_code}")
            return []
    except Exception as e:
        st.error(f"Error: {e}")
        return []

# Function to create the relative map
def plot_relative_map(df):
    # Create Plotly figure
    fig = go.Figure()

    # Add track bounds as a rectangle
    fig.add_trace(
        go.Scatter(
            x=[TRACK_BOUNDS["bottom_left"][0], TRACK_BOUNDS["bottom_right"][0], TRACK_BOUNDS["top_right"][0], TRACK_BOUNDS["top_left"][0], TRACK_BOUNDS["bottom_left"][0]],
            y=[TRACK_BOUNDS["bottom_left"][1], TRACK_BOUNDS["bottom_right"][1], TRACK_BOUNDS["top_right"][1], TRACK_BOUNDS["top_left"][1], TRACK_BOUNDS["bottom_left"][1]],
            mode="lines",
            line=dict(color="blue", width=2),
            name="Track Boundary",
        )
    )

    # Add data points
    fig.add_trace(
        go.Scatter(
            x=df["x"],
            y=df["z"],
            mode="markers",
            marker=dict(color="red", size=8),
            name="Robot Positions",
        )
    )

    # Set layout
    fig.update_layout(
        title="Relative Map of the Track",
        xaxis_title="X Coordinate",
        yaxis_title="Z Coordinate",
        xaxis=dict(range=[-1200, 1200]),
        yaxis=dict(range=[-900, 1200]),
        height=600,
        width=800,
    )

    return fig

# Streamlit app
st.title("Robot Position Tracker (Relative Map)")

# CSS for fixed-size scrollable container
st.markdown(
    """
    <style>
    .scrollable-container {
        height: 300px;
        overflow-y: scroll;
        border: 1px solid #ccc;
        padding: 10px;
        background-color: #f9f9f9;
        margin-bottom: 20px;
    }
    </style>
    """,
    unsafe_allow_html=True,
)

# Placeholder for real-time updates
placeholder = st.empty()

# Loop for real-time updates
while True:
    positions = fetch_positions()
    
    # Convert positions to a DataFrame
    if positions:
        df = pd.DataFrame(positions)

    with placeholder.container():
        st.header("Live Position Data")

        # Display position details in a scrollable container
        if positions:
            html_content = "<div class='scrollable-container'>"
            for pos in positions:
                html_content += (
                    f"<p>Robot ID: {pos['robot_id']}, X: {pos['x']}, Z: {pos['z']}, Theta: {pos['theta']}, Status: {pos['status']}</p>"
                )
            html_content += "</div>"
            st.markdown(html_content, unsafe_allow_html=True)
        else:
            st.info("No data available.")

        # Display the relative map
        if positions:
            st.write("Relative Map:")
            fig = plot_relative_map(df)
            st.plotly_chart(fig, use_container_width=True, key=f"plot_{time.time()}")

    # Wait for the next refresh
    time.sleep(REFRESH_INTERVAL)
