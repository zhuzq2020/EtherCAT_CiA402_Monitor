#pragma once

#include <QList>
#include <QMetaType>
#include <QString>
#include <QtGlobal>

struct AdapterInfo {
    QString name;
    QString description;
};
using AdapterInfoList = QList<AdapterInfo>;

struct SlaveInfo {
    int index = 0;
    QString name;
    quint32 vendorId = 0;
    quint32 productCode = 0;
    quint32 revision = 0;
    QString ethercatState;
    bool coeSupported = false;
    bool dcSupported = false;
};
using SlaveInfoList = QList<SlaveInfo>;

struct DriveSnapshot {
    int slave = 0;
    QString ethercatState;
    bool statusValid = false;
    quint16 statusWord = 0;
    QString cia402State;
    bool controlValid = false;
    quint16 controlWord = 0;
    bool modeCommandValid = false;
    qint8 modeCommand = 0;
    bool modeDisplayValid = false;
    qint8 modeDisplay = 0;
    QString modeText;
    bool positionValid = false;
    qint32 position = 0;
    bool velocityValid = false;
    qint32 velocity = 0;
    bool torqueValid = false;
    qint16 torque = 0;
    bool errorCodeValid = false;
    quint16 errorCode = 0;
    bool errorRegisterValid = false;
    quint8 errorRegister = 0;
};
using DriveSnapshotList = QList<DriveSnapshot>;

Q_DECLARE_METATYPE(AdapterInfo)
Q_DECLARE_METATYPE(AdapterInfoList)
Q_DECLARE_METATYPE(SlaveInfo)
Q_DECLARE_METATYPE(SlaveInfoList)
Q_DECLARE_METATYPE(DriveSnapshot)
Q_DECLARE_METATYPE(DriveSnapshotList)

