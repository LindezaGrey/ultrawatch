#!/usr/bin/env python3
"""Create the high-contrast QGIS project for the Hessen offline map."""

from __future__ import annotations

import argparse
import sqlite3
from pathlib import Path

from qgis.PyQt.QtGui import QColor, QFont
from qgis.core import (
    Qgis,
    QgsApplication,
    QgsFillSymbol,
    QgsLineSymbol,
    QgsMarkerSymbol,
    QgsPalLayerSettings,
    QgsProject,
    QgsSimpleLineSymbolLayer,
    QgsProperty,
    QgsTextBufferSettings,
    QgsTextFormat,
    QgsVectorLayer,
    QgsVectorLayerSimpleLabeling,
)


BACKGROUND_COLOR = "#f6f3eb"
LABEL_COLOR = "#20242a"
LABEL_BUFFER_COLOR = "#ffffff"

POLYGON_STYLES = {
    "gewaesserflaeche_bdlm": ("#b9dbea", "#6ca8c2"),
    "vegetationsflaeche_bdlm": ("#dce9d4", "#a8c39a"),
    "siedlungsflaeche_bdlm": ("#eee9e2", "#c6bdb3"),
    "bauwerksflaeche_bdlm": ("#d7d1c8", "#a69c90"),
    "verkehrsflaeche_bdlm": ("#f9f7f2", "#b8afa5"),
    "reliefflaeche_bdlm": ("#f0e9d9", "#cdbf9f"),
    "besondere_flaeche_bdlm": ("#eee7d8", "#b8aa8b"),
    "weitere_nutzung_flaeche_bdlm": ("#ece7de", "#bbb2a7"),
}

POLYGON_MINIMUM_SCALES = {
    "bauwerksflaeche_bdlm": 75_000,
    "verkehrsflaeche_bdlm": 75_000,
    "besondere_flaeche_bdlm": 100_000,
}

ROAD_LAYERS = (
    {
        "name": "Motorways",
        "filter": '"objektart" = \'Strassenachse\' AND "klasse" = \'Bundesautobahn\'',
        "minimum_scale": 0,
        "casing": "#62584d",
        "center": "#f2b84b",
        "casing_width": 1.65,
        "center_width": 1.05,
    },
    {
        "name": "Federal roads",
        "filter": '"objektart" = \'Strassenachse\' AND "klasse" = \'Bundesstraße\'',
        "minimum_scale": 2_500_000,
        "casing": "#68625c",
        "center": "#ffffff",
        "casing_width": 1.45,
        "center_width": 0.90,
    },
    {
        "name": "State and district roads",
        "filter": '"objektart" = \'Strassenachse\' AND "klasse" IN '
        "('Landesstraße, Staatsstraße', 'Kreisstraße')",
        "minimum_scale": 500_000,
        "casing": "#837d75",
        "center": "#fff8e7",
        "casing_width": 1.15,
        "center_width": 0.66,
    },
    {
        "name": "Local roads",
        "filter": '"objektart" = \'Strassenachse\' AND "klasse" IN '
        "('Gemeindestraße', 'Sonstige öffentliche Straße')",
        "minimum_scale": 100_000,
        "casing": "#a39d96",
        "center": "#ffffff",
        "casing_width": 0.82,
        "center_width": 0.42,
    },
    {
        "name": "Paths and tracks",
        "filter": '"objektart" IN (\'Fahrwegachse\', \'WegPfadSteig\')',
        "minimum_scale": 50_000,
        "casing": "#8f806b",
        "center": None,
        "casing_width": 0.32,
        "center_width": 0,
    },
    {
        "name": "Railways",
        "filter": '"objektart" = \'Bahnstrecke\'',
        "minimum_scale": 500_000,
        "casing": "#555b64",
        "center": "#f6f3eb",
        "casing_width": 0.72,
        "center_width": 0.28,
    },
)

BOUNDARY_LAYERS = (
    ("State boundaries", '"klasse" = \'Grenze des Bundeslandes\'', 0, 0.75),
    (
        "District boundaries",
        '"klasse" = \'Grenze des Kreises / Kreisfreien Stadt / Region\'',
        1_000_000,
        0.45,
    ),
    ("Municipal boundaries", '"klasse" = \'Grenze der Gemeinde\'', 150_000, 0.28),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path, help="basemap.de GeoPackage")
    parser.add_argument("output", type=Path, help="output .qgs file")
    parser.add_argument(
        "--labels", type=Path, required=True, help="BKG GN250 point Shapefile"
    )
    return parser.parse_args()


def layer_names(source: Path) -> list[str]:
    with sqlite3.connect(source) as database:
        rows = database.execute(
            "SELECT table_name FROM gpkg_contents "
            "WHERE data_type = 'features' ORDER BY table_name"
        )
        return [row[0] for row in rows]


def set_minimum_scale(layer: QgsVectorLayer, scale: float) -> None:
    if scale <= 0:
        return
    layer.setScaleBasedVisibility(True)
    layer.setMinimumScale(scale)
    layer.setMaximumScale(0)


def road_symbol(
    casing: str,
    center: str | None,
    casing_width: float,
    center_width: float,
) -> QgsLineSymbol:
    symbol = QgsLineSymbol.createSimple(
        {"color": casing, "width": str(casing_width), "capstyle": "round"}
    )
    if center:
        center_layer = QgsSimpleLineSymbolLayer.create(
            {"line_color": center, "line_width": str(center_width), "capstyle": "round"}
        )
        if center_layer is None:
            raise RuntimeError("QGIS could not create a road center line")
        symbol.appendSymbolLayer(center_layer)
    return symbol


def load_layer(
    project: QgsProject,
    source: Path,
    table: str,
    display_name: str | None = None,
    subset: str | None = None,
) -> QgsVectorLayer:
    uri = f"{source}|layername={table}"
    layer = QgsVectorLayer(uri, display_name or table, "ogr")
    if not layer.isValid():
        raise RuntimeError(f"QGIS could not load layer: {table}")
    if subset and not layer.setSubsetString(subset):
        raise RuntimeError(f"QGIS rejected filter for {display_name or table}: {subset}")
    project.addMapLayer(layer, False)
    return layer


def add_polygon_layers(project: QgsProject, source: Path) -> list[QgsVectorLayer]:
    layers = []
    for table, (fill, stroke) in POLYGON_STYLES.items():
        layer = load_layer(project, source, table)
        outline_width = "0.12" if table in {
            "gewaesserflaeche_bdlm",
            "bauwerksflaeche_bdlm",
            "verkehrsflaeche_bdlm",
        } else "0"
        layer.renderer().setSymbol(
            QgsFillSymbol.createSimple(
                {
                    "color": fill,
                    "outline_color": stroke,
                    "outline_width": outline_width,
                }
            )
        )
        set_minimum_scale(layer, POLYGON_MINIMUM_SCALES.get(table, 0))
        layers.append(layer)
    return layers


def add_road_layers(project: QgsProject, source: Path) -> list[QgsVectorLayer]:
    layers = []
    for definition in ROAD_LAYERS:
        layer = load_layer(
            project,
            source,
            "verkehrslinie_bdlm",
            definition["name"],
            definition["filter"],
        )
        layer.renderer().setSymbol(
            road_symbol(
                definition["casing"],
                definition["center"],
                definition["casing_width"],
                definition["center_width"],
            )
        )
        set_minimum_scale(layer, definition["minimum_scale"])
        layers.append(layer)
    return layers


def add_reference_lines(project: QgsProject, source: Path) -> list[QgsVectorLayer]:
    layers = []
    water = load_layer(project, source, "gewaesserlinie_bdlm", "Waterways")
    water.renderer().setSymbol(
        QgsLineSymbol.createSimple({"color": "#5e9db7", "width": "0.42"})
    )
    set_minimum_scale(water, 150_000)
    layers.append(water)

    for name, subset, minimum_scale, width in BOUNDARY_LAYERS:
        layer = load_layer(
            project, source, "grenze_linie_bdlm", name, subset
        )
        layer.renderer().setSymbol(
            QgsLineSymbol.createSimple(
                {"color": "#7d7194", "width": str(width), "line_style": "dash"}
            )
        )
        set_minimum_scale(layer, minimum_scale)
        layers.append(layer)
    return layers


def add_label_layer(
    project: QgsProject,
    source: Path,
    name: str,
    subset: str,
    show_expression: str,
    size_expression: str,
) -> QgsVectorLayer:
    layer = QgsVectorLayer(str(source), name, "ogr")
    if not layer.isValid():
        raise RuntimeError(f"QGIS could not load label source: {source}")
    if not layer.setSubsetString(subset):
        raise RuntimeError(f"QGIS rejected label filter: {subset}")

    layer.renderer().setSymbol(
        QgsMarkerSymbol.createSimple(
            {"color": "0,0,0,0", "outline_color": "0,0,0,0", "size": "0"}
        )
    )
    text_format = QgsTextFormat()
    text_format.setFont(QFont("DejaVu Sans"))
    text_format.setSize(10)
    text_format.setColor(QColor(LABEL_COLOR))
    buffer = QgsTextBufferSettings()
    buffer.setEnabled(True)
    buffer.setSize(1.5)
    buffer.setColor(QColor(LABEL_BUFFER_COLOR))
    text_format.setBuffer(buffer)

    settings = QgsPalLayerSettings()
    settings.enabled = True
    settings.fieldName = "NAME"
    settings.placement = Qgis.LabelPlacement.OverPoint
    settings.setFormat(text_format)
    properties = settings.dataDefinedProperties()
    properties.setProperty(
        QgsPalLayerSettings.Property.Show,
        QgsProperty.fromExpression(show_expression),
    )
    properties.setProperty(
        QgsPalLayerSettings.Property.Size,
        QgsProperty.fromExpression(size_expression),
    )
    properties.setProperty(
        QgsPalLayerSettings.Property.Priority,
        QgsProperty.fromExpression(
            "CASE WHEN EWZ IS NULL THEN 1 "
            "ELSE scale_linear(EWZ, 0, 800000, 2, 10) END"
        ),
    )
    layer.setLabeling(QgsVectorLayerSimpleLabeling(settings))
    layer.setLabelsEnabled(True)
    project.addMapLayer(layer, False)
    return layer


def main() -> None:
    args = parse_args()
    source = args.source.resolve()
    labels = args.labels.resolve()
    output = args.output.resolve()
    if not source.is_file():
        raise SystemExit(f"GeoPackage not found: {source}")
    if not labels.is_file():
        raise SystemExit(f"GN250 label source not found: {labels}")
    output.parent.mkdir(parents=True, exist_ok=True)

    application = QgsApplication([], False)
    application.initQgis()
    project = QgsProject.instance()
    try:
        project.clear()
        project.setTitle("UltraWatch Hessen high-contrast day map")
        project.setCrs(project.crs().fromEpsgId(4326))
        project.setBackgroundColor(QColor(BACKGROUND_COLOR))
        project.setFileName(str(output))
        project.setFilePathStorage(Qgis.FilePathType.Relative)

        available_layers = set(layer_names(source))
        required_layers = set(POLYGON_STYLES) | {
            "verkehrslinie_bdlm",
            "gewaesserlinie_bdlm",
            "grenze_linie_bdlm",
        }
        missing_layers = sorted(required_layers - available_layers)
        if missing_layers:
            raise RuntimeError(f"GeoPackage is missing layers: {', '.join(missing_layers)}")

        polygon_layers = add_polygon_layers(project, source)
        road_layers = add_road_layers(project, source)
        reference_lines = add_reference_lines(project, source)

        root = project.layerTreeRoot()
        for layer in [*road_layers, *reference_lines, *polygon_layers]:
            root.addLayer(layer)

        municipality_labels = add_label_layer(
            project,
            labels,
            "City names",
            "BUNDESLAND = 'Hessen' AND OBA = 'AX_Gemeinde'",
            "CASE "
            "WHEN @map_scale > 5000000 THEN EWZ >= 100000 "
            "WHEN @map_scale > 1500000 THEN EWZ >= 30000 "
            "WHEN @map_scale > 500000 THEN EWZ >= 10000 "
            "ELSE TRUE END",
            "CASE WHEN EWZ >= 100000 THEN 12 "
            "WHEN EWZ >= 30000 THEN 10 ELSE 8 END",
        )
        locality_labels = add_label_layer(
            project,
            labels,
            "Locality names",
            "BUNDESLAND = 'Hessen' AND OBA = 'AX_Ortslage' "
            "AND NAME <> GEMEINDE",
            "@map_scale <= 300000",
            "8",
        )
        root.insertLayer(0, locality_labels)
        root.insertLayer(0, municipality_labels)

        if not project.write(str(output)):
            raise RuntimeError(f"QGIS could not write project: {output}")
        map_layer_count = len(polygon_layers) + len(road_layers) + len(reference_lines)
        print(f"Created {output} with {map_layer_count} map layers and 2 label layers")
    finally:
        project.clear()
        application.exitQgis()


if __name__ == "__main__":
    main()
