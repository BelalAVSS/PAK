#include "HistoricalTargetModel.h"

HistoricalTargetModel::HistoricalTargetModel(QObject* parent)
    : QAbstractListModel(parent)
{
}

int HistoricalTargetModel::rowCount(const QModelIndex& parent) const
{
    if (parent.isValid()) {
        return 0;
    }

    return _targets.count();
}

QVariant HistoricalTargetModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid()) {
        return QVariant();
    }

    const int row = index.row();
    if (row < 0 || row >= _targets.count()) {
        return QVariant();
    }

    const HistoricalTarget& t = _targets[row];

    switch (role) {
    case TypeRole:
        return t.type;
    case LatRole:
        return t.lat;
    case LonRole:
        return t.lon;
    case TimestampRole:
        return t.timestamp;
    default:
        return QVariant();
    }
}

QHash<int, QByteArray> HistoricalTargetModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[TypeRole] = "type";
    roles[LatRole] = "lat";
    roles[LonRole] = "lon";
    roles[TimestampRole] = "timestamp";
    return roles;
}

void HistoricalTargetModel::clear()
{
    beginResetModel();
    _targets.clear();
    endResetModel();
}

void HistoricalTargetModel::setTargets(const QList<HistoricalTarget>& targets)
{
    beginResetModel();
    _targets = targets;
    endResetModel();
}

void HistoricalTargetModel::addTarget(const HistoricalTarget& target)
{
    const int newRow = _targets.count();

    beginInsertRows(QModelIndex(), newRow, newRow);
    _targets.append(target);
    endInsertRows();
}
