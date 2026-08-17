#!/usr/bin/env python3
"""Create the minimal QGIS project for the Hessen offline-map render."""

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
    QgsProperty,
    QgsTextBufferSettings,
    QgsTextFormat,
    QgsVectorLayer,
    QgsVectorLayerSimpleLabeling,
    QgsWkbTypes,
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


def colors(name: str) -> tuple[str, str]:
    if name.startswith("gewaesser"):
        return "#102b42", "#41a9db"
    if name.startswith("vegetation"):
        return "#152e25", "#356a52"
    if name.startswith("siedlung") or name.startswith("bauwerk"):
        return "#272a31", "#707784"
    if name.startswith("verkehr"):
        return "#24262b", "#d5d8df"
    if name.startswith("grenze"):
        return "#171724", "#7772a8"
    return "#171b20", "#59616b"


def apply_style(layer: QgsVectorLayer, name: str) -> None:
    fill, stroke = colors(name)
    geometry = QgsWkbTypes.geometryType(layer.wkbType())
    if geometry == QgsWkbTypes.PolygonGeometry:
        symbol = QgsFillSymbol.createSimple(
            {"color": fill, "outline_color": stroke, "outline_width": "0.15"}
        )
    elif geometry == QgsWkbTypes.LineGeometry:
        width = "0.65" if name.startswith("verkehr") else "0.35"
        symbol = QgsLineSymbol.createSimple({"color": stroke, "width": width})
    else:
        symbol = QgsMarkerSymbol.createSimple(
            {"color": stroke, "outline_color": fill, "size": "1.2"}
        )
    layer.renderer().setSymbol(symbol)


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
    text_format.setColor(QColor("#f2f5f8"))
    buffer = QgsTextBufferSettings()
    buffer.setEnabled(True)
    buffer.setSize(1.2)
    buffer.setColor(QColor("#05080c"))
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
    try:
        project = QgsProject.instance()
        project.clear()
        project.setTitle("UltraWatch Hessen debug map")
        project.setCrs(project.crs().fromEpsgId(4326))
        project.setBackgroundColor(QColor("#05080c"))
        project.setFilePathStorage(Qgis.FilePathType.Relative)

        layers: list[QgsVectorLayer] = []
        for name in layer_names(source):
            uri = f"{source}|layername={name}"
            layer = QgsVectorLayer(uri, name, "ogr")
            if not layer.isValid():
                raise RuntimeError(f"QGIS could not load layer: {name}")
            apply_style(layer, name)
            project.addMapLayer(layer, False)
            layers.append(layer)

        root = project.layerTreeRoot()
        for geometry in (
            QgsWkbTypes.PolygonGeometry,
            QgsWkbTypes.LineGeometry,
            QgsWkbTypes.PointGeometry,
        ):
            for layer in layers:
                if QgsWkbTypes.geometryType(layer.wkbType()) == geometry:
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
        print(f"Created {output} with {len(layers)} map layers and 2 label layers")
    finally:
        application.exitQgis()


if __name__ == "__main__":
    main()
