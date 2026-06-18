import folium # type: ignore
import webbrowser
import os


class MapView3D:
    def __init__(self, start_lat: float, start_lon: float):
        self.map = folium.Map(location=[start_lat, start_lon], zoom_start=18)
        self.output_file = "telemetry_map.html"
        self.page_opened = False

    def update_marker(self, lat: float, lon: float, alt: float):
        self.map = folium.Map(location=[lat, lon], zoom_start=18)
        folium.Marker(
            location=[lat, lon],
            popup=f"Altitude: {alt:.2f} m",
            icon=folium.Icon(color="blue", icon="info-sign")
        ).add_to(self.map)

    def show_map(self):
        # Save map to HTML
        self.map.save(self.output_file)

        # Inject meta-refresh into HTML (once)
        with open(self.output_file, "r+", encoding="utf-8") as f:
            contents = f.read()
            if '<meta http-equiv="refresh"' not in contents:
                # Add meta-refresh right after <head>
                refreshed = contents.replace(
                    "<head>",
                    '<head>\n<meta http-equiv="refresh" content="0.5">'
                )
                f.seek(0)
                f.write(refreshed)
                f.truncate()

        if not self.page_opened:
            webbrowser.open('file://' + os.path.realpath(self.output_file))
            self.page_opened = True
