import streamlit as st
from streamlit_folium import st_folium
import folium
from folium import plugins
import json
import geocoder

g = geocoder.ip('me')

if g.ok:
    # g.latlng is a list containing [latitude, longitude]
    latitude = g.latlng[0]
    longitude = g.latlng[1]

def export(output):
    if output and output.get("all_drawings"):
        coord = output["all_drawings"][0]['geometry']['coordinates'][0][:-1]
        for i in range(len(coord)):
            coord[i] = [coord[i][1], coord[i][0]]
        print (coord)
        with open("/home/ayush/drone-ws-code/survey/survey/details.json", "w") as file:
            json.dump(coord, file)
    #    geojson = json.dumps(output["all_drawings"], indent=2)

# --- Title ---
col1, col2 = st.columns([10, 1])  # adjust ratio as needed

# --- Page setup ---
st.set_page_config(
    page_title="Sentry Map",
    page_icon="🗺️",
    layout="wide",
    initial_sidebar_state="collapsed"
)

# --- Hide default Streamlit elements and reduce padding ---
hide_streamlit_style = """
    <style>
    #MainMenu {visibility: hidden;}
    footer {visibility: hidden;}
    header {visibility: hidden;}
    /* Remove top padding of main content container */
    .css-18e3th9 {padding-top: 0rem; padding-bottom: 0rem;}
    /* Optional: reduce spacing between elements */
    .block-container {padding-top: 0.5rem; padding-bottom: 0rem;}
    </style>
"""
st.markdown(hide_streamlit_style, unsafe_allow_html=True)

# --- Map setup ---
m = folium.Map(location=[12.840151, 80.151165], zoom_start=15, width='100%', height='500')

# --- Add Draw plugin ---
draw = plugins.Draw(
    export=False,
    draw_options={
        'polygon': False,
        'polyline': False,
        'circle': False,
        'marker': False,
        'circlemarker': False,
        'rectangle': True
    },
    edit_options={'edit': True}
)
draw.add_to(m)

# --- Display the map ---
output = st_folium(m, width=1200, height=600)


with col1:
    st.markdown(
        "<h2 style='text-align:center; font-size:20px; margin:0;padding-left:100px;'>Area Selection Interface</h2>",
        unsafe_allow_html=True,)

with col2:
    if st.button("Send"):
        export(output)