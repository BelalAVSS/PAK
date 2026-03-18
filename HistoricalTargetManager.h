#pragma once

#include <QObject>
#include <QString>
#include "HistoricalTargetModel.h"
#include <QList>

// THE FILE WAS ADDED BY BELAL

class HistoricalTargetModel;

Q_MOC_INCLUDE("HistoricalTargetModel.h")

class HistoricalTargetManager : public QObject
{
    Q_OBJECT

    Q_PROPERTY(HistoricalTargetModel* model READ model CONSTANT)
    Q_PROPERTY(int loadedCount READ loadedCount NOTIFY loadedCountChanged)
    Q_PROPERTY(QString filePath READ filePath CONSTANT)

public:
    explicit HistoricalTargetManager(QObject* parent = nullptr);

    HistoricalTargetModel* model() const { return _model; }
    int loadedCount() const;
    QString filePath() const;

    Q_INVOKABLE bool saveCurrentTargets();
    Q_INVOKABLE bool loadTargets();
    Q_INVOKABLE bool clearTargets();
    Q_INVOKABLE bool hideTargets();

signals:
    void loadedCountChanged();
    void errorOccurred(const QString& message);
    void saveSucceeded(int count);
    void loadSucceeded(int count);

private:
    HistoricalTargetModel* _model = nullptr;
    bool _writeTargetsToFile(const QList<HistoricalTarget>& targets);
    QString _typeFromCallsign(const QString& callsign) const;
    QList<HistoricalTarget> _snapshotCurrentTargets() const;
    QList<HistoricalTarget> _readTargetsFromFile() const;
};
