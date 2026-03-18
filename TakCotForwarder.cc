#include "take_cot_forward.h"

#include "ADSBVehicleManager.h"
#include "QmlObjectListModel.h"

#include <QDateTime>
#include <QGeoCoordinate>
#include <QHostAddress>
#include <QXmlStreamWriter>
#include <QBuffer>
#include <QDebug>

TakCotForwarder::TakCotForwarder(QObject* parent)
    : QObject(parent)
{
    _timer.setInterval(1000);   // every 1 second
    connect(&_timer, &QTimer::timeout, this, &TakCotForwarder::_tick);
    qDebug() << "[TAK] TakCotForwarder created";
}

void TakCotForwarder::setEnabled(bool enabled)
{
    if (_enabled == enabled) {
        return;
    }

    _enabled = enabled;

    if (_enabled) {
        _timer.start();
    } else {
        _timer.stop();
    }
}

QString TakCotForwarder::_typeFromCallsign(const QString& callsign) const
{
    const QString cs = callsign.trimmed().toUpper();

    if (cs.startsWith(QStringLiteral("PERSON")) || cs.contains(QStringLiteral("PERSON"))) {
        return QStringLiteral("person");
    }

    if (cs.startsWith(QStringLiteral("CAR")) || cs.contains(QStringLiteral("CAR"))) {
        return QStringLiteral("car");
    }

    return QStringLiteral("unknown");
}

QString TakCotForwarder::_takTypeFromLabel(const QString& label) const
{
    // Keep first pass simple.
    // We can refine exact TAK symbol types later.
    if (label == QStringLiteral("person")) {
        return QStringLiteral("a-f-G-U-C");
    }

    if (label == QStringLiteral("car")) {
        return QStringLiteral("a-f-G-E-V-C");
    }

    return QStringLiteral("a-f-G-U-C");
}

QString TakCotForwarder::_cotTimeString(int offsetSec) const
{
    const QDateTime t = QDateTime::currentDateTimeUtc().addSecs(offsetSec);
    return t.toString(QStringLiteral("yyyy-MM-dd'T'HH:mm:ss.zzz'Z'"));
}

QByteArray TakCotForwarder::_buildCotXml(
    const QString& uid,
    const QString& callsign,
    const QString& label,
    double lat,
    double lon,
    double haeMeters
) const
{
    QByteArray payload;
    QBuffer buffer(&payload);
    buffer.open(QIODevice::WriteOnly);

    QXmlStreamWriter xml(&buffer);
    xml.setAutoFormatting(false);

    xml.writeStartDocument();

    xml.writeStartElement(QStringLiteral("event"));
    xml.writeAttribute(QStringLiteral("version"), QStringLiteral("2.0"));
    xml.writeAttribute(QStringLiteral("uid"), uid);
    xml.writeAttribute(QStringLiteral("type"), _takTypeFromLabel(label));
    xml.writeAttribute(QStringLiteral("how"), QStringLiteral("m-g"));
    xml.writeAttribute(QStringLiteral("time"),  _cotTimeString(0));
    xml.writeAttribute(QStringLiteral("start"), _cotTimeString(0));
    xml.writeAttribute(QStringLiteral("stale"), _cotTimeString(10));

    xml.writeStartElement(QStringLiteral("point"));
    xml.writeAttribute(QStringLiteral("lat"), QString::number(lat, 'f', 7));
    xml.writeAttribute(QStringLiteral("lon"), QString::number(lon, 'f', 7));
    xml.writeAttribute(QStringLiteral("hae"), QString::number(haeMeters, 'f', 1));
    xml.writeAttribute(QStringLiteral("ce"), QStringLiteral("20.0"));
    xml.writeAttribute(QStringLiteral("le"), QStringLiteral("20.0"));
    xml.writeEndElement();

    xml.writeStartElement(QStringLiteral("detail"));

    xml.writeStartElement(QStringLiteral("contact"));
    xml.writeAttribute(QStringLiteral("callsign"), callsign);
    xml.writeEndElement();

    xml.writeStartElement(QStringLiteral("remarks"));
    xml.writeCharacters(QStringLiteral("Forwarded from QGC Android"));
    xml.writeEndElement();

    xml.writeEndElement(); // detail
    xml.writeEndElement(); // event
    xml.writeEndDocument();

    buffer.close();
    return payload;
}

void TakCotForwarder::_tick()
{
    if (!_enabled) {
        return;
    }

    qDebug() << "[TAK] tick";

    const QByteArray payload = _buildCotXml(
        QStringLiteral("QGC-ANDROID-TEST-1"),
        QStringLiteral("QGC-ANDROID-TEST-1"),
        QStringLiteral("person"),
        45.4215000,
        -75.6972000,
        70.0
    );

    qDebug() << "[TAK] sending fixed test marker to" << _host << _port;
    _udp.writeDatagram(payload, QHostAddress(_host), _port);

    return;
}

// void TakCotForwarder::_tick()
// {
//     qDebug() << "[TAK] tick";
//     if (!_enabled) {
//         return;
//     }

//     ADSBVehicleManager* adsbManager = ADSBVehicleManager::instance();
//     if (!adsbManager) {
//         return;
//     }

//     const QmlObjectListModel* adsbVehicles = adsbManager->adsbVehicles();
//     if (!adsbVehicles) {
//         return;
//     }

//     QmlObjectListModel* adsbVehiclesMutable = const_cast<QmlObjectListModel*>(adsbVehicles);

//     for (int i = 0; i < adsbVehiclesMutable->count(); ++i) {
//         QObject* obj = adsbVehiclesMutable->get(i);
//         if (!obj) {
//             continue;
//         }

//         const QString callsign = obj->property("callsign").toString().trimmed();
//         if (callsign.isEmpty()) {
//             continue;
//         }

//         const QVariant coordVar = obj->property("coordinate");
//         if (!coordVar.isValid()) {
//             continue;
//         }

//         const QGeoCoordinate coord = coordVar.value<QGeoCoordinate>();
//         if (!coord.isValid()) {
//             continue;
//         }

//         const QString label = _typeFromCallsign(callsign);

//         // First pass UID: based on callsign.
//         // Later we can switch to ICAO if that property exists.
//         const QString uid = QStringLiteral("qgc-") + callsign.toUpper();

//         double haeMeters = coord.altitude();
//         if (!qIsFinite(haeMeters)) {
//             haeMeters = 0.0;
//         }

//         const QByteArray payload = _buildCotXml(
//             uid,
//             callsign,
//             label,
//             coord.latitude(),
//             coord.longitude(),
//             haeMeters
//         );
//         qDebug() << "[TAK] sending payload to" << _host << _port;
//         _udp.writeDatagram(payload, QHostAddress(_host), _port);

//         qDebug() << "[TAK] sent" << callsign
//                  << coord.latitude()
//                  << coord.longitude()
//                  << "uid =" << uid;
//     }
// }
