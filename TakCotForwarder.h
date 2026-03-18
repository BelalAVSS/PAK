#pragma once

#include <QObject>
#include <QTimer>
#include <QUdpSocket>

class TakCotForwarder : public QObject
{
    Q_OBJECT
public:
    explicit TakCotForwarder(QObject* parent = nullptr);

    void setEnabled(bool enabled);
    bool enabled() const { return _enabled; }

    void setHost(const QString& host) { _host = host; }
    void setPort(quint16 port) { _port = port; }

private slots:
    void _tick();

private:
    QString _typeFromCallsign(const QString& callsign) const;
    QString _takTypeFromLabel(const QString& label) const;
    QByteArray _buildCotXml(
        const QString& uid,
        const QString& callsign,
        const QString& label,
        double lat,
        double lon,
        double haeMeters
    ) const;

    QString _cotTimeString(int offsetSec = 0) const;

private:
    bool        _enabled = false;
    QString     _host = QStringLiteral("216.83.28.209");
    quint16     _port = 14550;
    QTimer      _timer;
    QUdpSocket  _udp;
};
