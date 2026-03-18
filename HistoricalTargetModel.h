#pragma once

#include <QAbstractListModel>
#include <QList>
#include <QString>

struct HistoricalTarget
{
    QString type;
    double lat = 0.0;
    double lon = 0.0;
    QString timestamp;
};

class HistoricalTargetModel : public QAbstractListModel
{
    Q_OBJECT

public:
    enum Roles {
        TypeRole = Qt::UserRole + 1,
        LatRole,
        LonRole,
        TimestampRole
    };
    Q_ENUM(Roles)

    explicit HistoricalTargetModel(QObject* parent = nullptr);

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override;

    void clear();
    void setTargets(const QList<HistoricalTarget>& targets);
    void addTarget(const HistoricalTarget& target);

    const QList<HistoricalTarget>& targets() const { return _targets; }

private:
    QList<HistoricalTarget> _targets;
};
