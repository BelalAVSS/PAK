#include "HistoricalTargetManager.h"
#include "HistoricalTargetModel.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStandardPaths>

#include "ADSBVehicleManager.h"
#include "QmlObjectListModel.h"

#include <QDateTime>
#include <QGeoCoordinate>
#include <QDebug>

// THE FILES WAS ADDED BY BELAL
HistoricalTargetManager::HistoricalTargetManager(QObject* parent)
    : QObject(parent)
    , _model(new HistoricalTargetModel(this))
{
}

int HistoricalTargetManager::loadedCount() const
{
    return _model ? _model->rowCount() : 0;
}

QString HistoricalTargetManager::filePath() const
{
    const QString dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    return dir + QStringLiteral("/historical_targets.json");
}

QString HistoricalTargetManager::_typeFromCallsign(const QString& callsign) const
{
    const QString cs = callsign.trimmed().toUpper();
    qDebug() << "[History] callsign raw =" << callsign << " normalized =" << cs;

    if (cs.startsWith(QStringLiteral("PERSON")) || cs.contains(QStringLiteral("PERSON"))) {
        return QStringLiteral("person");
    }

    if (cs.startsWith(QStringLiteral("CAR")) || cs.contains(QStringLiteral("CAR"))) {
        return QStringLiteral("car");
    }

    return QString();
}

QList<HistoricalTarget> HistoricalTargetManager::_snapshotCurrentTargets() const
{
    QList<HistoricalTarget> targets;

    ADSBVehicleManager* adsbManager = ADSBVehicleManager::instance();
    if (!adsbManager) {
        qDebug() << "[History] ADSB manager is null";
        return targets;
    }

    const QmlObjectListModel* adsbVehicles = adsbManager->adsbVehicles();
    if (!adsbVehicles) {
        qDebug() << "[History] ADSB vehicles model is null";
        return targets;
    }

    QmlObjectListModel* adsbVehiclesMutable = const_cast<QmlObjectListModel*>(adsbVehicles);

    qDebug() << "[History] ADSB count =" << adsbVehiclesMutable->count();

    const QString nowIso = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);

    for (int i = 0; i < adsbVehiclesMutable->count(); ++i) {
        QObject* obj = adsbVehiclesMutable->get(i);
        if (!obj) {
            qDebug() << "[History] null ADSB object at index" << i;
            continue;
        }

        const QString callsign = obj->property("callsign").toString();
        const QString type = _typeFromCallsign(callsign);
        qDebug() << "[History] index" << i << "callsign =" << callsign << "type =" << type;

        if (type.isEmpty()) {
            qDebug() << "[History] skipped because type is empty";
            continue;
        }

        const QVariant coordVar = obj->property("coordinate");
        qDebug() << "[History] coordinate valid =" << coordVar.isValid() << "typeName =" << coordVar.typeName();

        if (!coordVar.isValid()) {
            qDebug() << "[History] skipped because coordinate is invalid";
            continue;
        }

        const QGeoCoordinate coord = coordVar.value<QGeoCoordinate>();
        qDebug() << "[History] coord valid =" << coord.isValid()
                 << "lat =" << coord.latitude()
                 << "lon =" << coord.longitude();

        if (!coord.isValid()) {
            qDebug() << "[History] skipped because QGeoCoordinate is invalid";
            continue;
        }

        HistoricalTarget t;
        t.type = type;
        t.lat = coord.latitude();
        t.lon = coord.longitude();
        t.timestamp = nowIso;

        qDebug() << "[History] saving target:" << t.type << t.lat << t.lon << t.timestamp;
        targets.append(t);
    }

    qDebug() << "[History] snapshot count =" << targets.count();
    return targets;
}

QList<HistoricalTarget> HistoricalTargetManager::_readTargetsFromFile() const
{
    QList<HistoricalTarget> targets;

    QFile f(filePath());

    if (!f.exists()) {
        return targets;
    }

    if (!f.open(QIODevice::ReadOnly)) {
        qWarning() << "[History] _readTargetsFromFile: failed to open file:" << filePath();
        return targets;
    }

    const QByteArray raw = f.readAll();
    f.close();

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(raw, &parseError);

    if (parseError.error != QJsonParseError::NoError) {
        qWarning() << "[History] _readTargetsFromFile: JSON parse error =" << parseError.errorString();
        return targets;
    }

    if (!doc.isArray()) {
        qWarning() << "[History] _readTargetsFromFile: JSON root is not an array";
        return targets;
    }

    const QJsonArray arr = doc.array();

    for (const QJsonValue& value : arr) {
        if (!value.isObject()) {
            continue;
        }

        const QJsonObject obj = value.toObject();

        HistoricalTarget t;
        t.type = obj.value(QStringLiteral("type")).toString().trimmed();
        t.lat = obj.value(QStringLiteral("lat")).toDouble();
        t.lon = obj.value(QStringLiteral("lon")).toDouble();
        t.timestamp = obj.value(QStringLiteral("timestamp")).toString().trimmed();

        if (t.type.isEmpty()) {
            continue;
        }

        targets.append(t);
    }

    qWarning() << "[History] _readTargetsFromFile: loaded" << targets.count() << "targets";
    return targets;
}

bool HistoricalTargetManager::saveCurrentTargets()
{
    const QList<HistoricalTarget> currentTargets = _snapshotCurrentTargets();
    QList<HistoricalTarget> existingTargets = _readTargetsFromFile();

    existingTargets.append(currentTargets);

    qDebug() << "[History] saveCurrentTargets count =" << currentTargets.count();
    qDebug() << "[History] file path =" << filePath();

    if (!_writeTargetsToFile(existingTargets)) {
        qDebug() << "[History] write failed";
        return false;
    }

    qDebug() << "[History] write succeeded";
    emit saveSucceeded(currentTargets.count());
    return true;
}

bool HistoricalTargetManager::loadTargets()
{
    qWarning() << "[History] loading from file =" << filePath();

    if (!_model) {
        qWarning() << "[History] load failed: model is null";
        emit errorOccurred(QStringLiteral("Historical model is null"));
        return false;
    }

    QFile f(filePath());

    if (!f.exists()) {
        qWarning() << "[History] file does not exist, clearing model";
        _model->clear();
        emit loadedCountChanged();
        emit loadSucceeded(0);
        return true;
    }

    if (!f.open(QIODevice::ReadOnly)) {
        qWarning() << "[History] load failed: could not open file";
        emit errorOccurred(QStringLiteral("Failed to open history file for reading"));
        return false;
    }

    const QByteArray raw = f.readAll();
    f.close();

    qWarning() << "[History] raw file bytes =" << raw;
    qWarning() << "[History] raw file size =" << raw.size();

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(raw, &parseError);

    if (parseError.error != QJsonParseError::NoError) {
        qWarning() << "[History] JSON parse error =" << parseError.errorString();
        emit errorOccurred(QStringLiteral("Failed to parse history JSON"));
        return false;
    }

    if (!doc.isArray()) {
        qWarning() << "[History] JSON root is not an array";
        emit errorOccurred(QStringLiteral("History JSON root is not an array"));
        return false;
    }

    QList<HistoricalTarget> targets;
    const QJsonArray arr = doc.array();

    qWarning() << "[History] JSON array size =" << arr.size();

    for (const QJsonValue& value : arr) {
        if (!value.isObject()) {
            qWarning() << "[History] skipped non-object JSON value";
            continue;
        }

        const QJsonObject obj = value.toObject();

        HistoricalTarget t;
        t.type = obj.value(QStringLiteral("type")).toString().trimmed();
        t.lat = obj.value(QStringLiteral("lat")).toDouble();
        t.lon = obj.value(QStringLiteral("lon")).toDouble();
        t.timestamp = obj.value(QStringLiteral("timestamp")).toString().trimmed();

        qWarning() << "[History] parsed target:" << t.type << t.lat << t.lon << t.timestamp;

        if (t.type.isEmpty()) {
            qWarning() << "[History] skipped parsed target because type is empty";
            continue;
        }

        targets.append(t);
    }

    qWarning() << "[History] final loaded target count =" << targets.count();

    _model->setTargets(targets);

    emit loadedCountChanged();
    emit loadSucceeded(targets.count());
    return true;
}

bool HistoricalTargetManager::hideTargets()
{
    _model->clear();
    emit loadedCountChanged();
    return true;
}

bool HistoricalTargetManager::clearTargets()
{

    if (!_writeTargetsToFile({})) {
        return false;
    }

    _model->clear();
    emit loadedCountChanged();
    return true;
}



bool HistoricalTargetManager::_writeTargetsToFile(const QList<HistoricalTarget>& existingTargets)
{
    QJsonArray arr;

    for (const HistoricalTarget& t : existingTargets) {
        QJsonObject obj;
        obj[QStringLiteral("type")] = t.type;
        obj[QStringLiteral("lat")] = t.lat;
        obj[QStringLiteral("lon")] = t.lon;
        obj[QStringLiteral("timestamp")] = t.timestamp;
        qDebug() << "[History] writing:" << t.type << t.lat << t.lon << t.timestamp;
        arr.append(obj);
    }

    QJsonDocument doc(arr);

    QFileInfo fi(filePath());
    QDir().mkpath(fi.absolutePath());

    QFile f(filePath());
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        emit errorOccurred(QStringLiteral("Failed to open history file for writing"));
        return false;
    }

    f.write(doc.toJson(QJsonDocument::Indented));
    f.close();
    return true;
}
