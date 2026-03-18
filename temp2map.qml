


    // Add historical targets to the map
MapItemView {
    model: QGroundControl.historicalTargetManager.model

    delegate: MapQuickItem {
        coordinate: QtPositioning.coordinate(lat, lon)
        z: QGroundControl.zOrderMapItems

        anchorPoint.x: 30
        anchorPoint.y: 14

        sourceItem: Item {
            width: 60
            height: 42

            Column {
                anchors.horizontalCenter: parent.horizontalCenter
                spacing: 2

                Rectangle {
                    width: 28
                    height: 28
                    radius: 14
                    color: type === "person" ? "#8844AAFF" : "#88FFAA44"
                    border.width: 1
                    border.color: "white"
                    opacity: 0.75
                    anchors.horizontalCenter: parent.horizontalCenter

                    Text {
                        anchors.centerIn: parent
                        text: type === "person" ? "P" : "C"
                        color: "white"
                        font.bold: true
                    }
                }

                Text {
                    text: type
                    color: "white"
                    font.bold: true
                    font.pixelSize: 11
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Text {
                    text : timestamp
                    color : "white"
                    font.bold : true
                    font.pixelSize: 10
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                }
            }
        }
    }
}


