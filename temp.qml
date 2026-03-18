 Rectangle {
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.margins: 12
        width: 150
        height: controlsColumn.implicitHeight + 16
        radius: 8
        color: "#66000000"
        border.color: "#66FFFFFF"
        border.width: 1

        Column {
            id: controlsColumn
            anchors.fill: parent
            anchors.margins: 8
            spacing: 8

            QGCLabel {
                width: parent.width -20
                anchors.horizontalCenter: parent.horizontalCenter
                text: "Previous Targets"
                font.bold: true
            }


            QGCButton {
                width: parent.width -20
                anchors.horizontalCenter: parent.horizontalCenter
                text: "Save Targets"
                onClicked: QGroundControl.historicalTargetManager.saveCurrentTargets()
            }

            QGCButton {
                width: parent.width -20
                anchors.horizontalCenter: parent.horizontalCenter
                text: "Load Previous"
                onClicked: QGroundControl.historicalTargetManager.loadTargets()
            }

            QGCButton {
                width: parent.width - 20
                anchors.horizontalCenter: parent.horizontalCenter
                text: "Hide Targets"
                onClicked: QGroundControl.historicalTargetManager.hideTargets()
            }

            QGCButton {
                width: parent.width -20
                anchors.horizontalCenter: parent.horizontalCenter
                text: "Clear History"
                onClicked: QGroundControl.historicalTargetManager.clearTargets()
            }

            QGCLabel {
                anchors.horizontalCenter: parent.horizontalCenter
                text: "Count: " + QGroundControl.historicalTargetManager.loadedCount
            }
        }
    }
}
