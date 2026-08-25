#include "EthercatWorker.h"

#include "Cia402State.h"

#include <QDateTime>
#include <QTimer>

#include <cmath>
#include <cstring>

EthercatWorker::EthercatWorker(QObject *parent)
    : QObject(parent)
{
}

EthercatWorker::~EthercatWorker()
{
    closeInterface();
}

void EthercatWorker::discoverAdapters()
{
    AdapterInfoList result;
    result.append({QString::fromLatin1(kDemoInterface), tr("演示模式（无需 EtherCAT 硬件）")});

    ec_adaptert *adapters = ec_find_adapters();
    for (ec_adaptert *item = adapters; item != nullptr; item = item->next) {
        result.append({QString::fromLocal8Bit(item->name), QString::fromLocal8Bit(item->desc)});
    }
    ec_free_adapters(adapters);
    emit adaptersDiscovered(result);
}

void EthercatWorker::scan(const QString &interfaceName)
{
    ensurePollTimer();
    closeInterface();
    emit logMessage(tr("开始扫描接口: %1").arg(interfaceName));

    if (interfaceName == QLatin1String(kDemoInterface)) {
        scanDemo();
        return;
    }

    std::memset(&context_, 0, sizeof(context_));
    const QByteArray interfaceBytes = interfaceName.toLocal8Bit();
    if (ecx_init(&context_, interfaceBytes.constData()) <= 0) {
        emit connectionChanged(false,
            tr("无法打开 %1；请检查网卡名和 CAP_NET_RAW/CAP_NET_ADMIN 权限").arg(interfaceName));
        return;
    }
    initialized_ = true;

    const int count = ecx_config_init(&context_);
    if (count <= 0) {
        emit connectionChanged(false, tr("未发现 EtherCAT 从站"));
        closeInterface();
        return;
    }

    ecx_readstate(&context_);
    SlaveInfoList slaves;
    for (int i = 1; i <= context_.slavecount; ++i) {
        const ec_slavet &source = context_.slavelist[i];
        SlaveInfo info;
        info.index = i;
        info.name = QString::fromLocal8Bit(source.name);
        info.vendorId = source.eep_man;
        info.productCode = source.eep_id;
        info.revision = source.eep_rev;
        info.ethercatState = ethercatStateName(source.state);
        info.coeSupported = (source.mbx_proto & ECT_MBXPROT_COE) != 0;
        info.dcSupported = source.hasdc != 0;
        slaves.append(info);
    }

    emit scanCompleted(slaves);
    emit connectionChanged(true, tr("已连接，发现 %1 个从站（仅读监控）").arg(count));
    pollTimer_->start();
    poll();
}

void EthercatWorker::stop()
{
    const bool wasActive = initialized_ || demo_;
    closeInterface();
    if (wasActive) emit connectionChanged(false, tr("已停止监控"));
}

void EthercatWorker::setPollInterval(const int milliseconds)
{
    pollIntervalMs_ = qBound(100, milliseconds, 10000);
    if (pollTimer_ != nullptr) pollTimer_->setInterval(pollIntervalMs_);
}

void EthercatWorker::poll()
{
    if (demo_) {
        pollDemo();
        return;
    }
    if (!initialized_) return;

    ecx_readstate(&context_);
    DriveSnapshotList result;
    for (int slave = 1; slave <= context_.slavecount; ++slave) {
        DriveSnapshot snapshot;
        snapshot.slave = slave;
        snapshot.ethercatState = ethercatStateName(context_.slavelist[slave].state);

        if ((context_.slavelist[slave].mbx_proto & ECT_MBXPROT_COE) != 0) {
            snapshot.statusValid = readSdo(slave, 0x6041, 0, snapshot.statusWord);
            if (snapshot.statusValid) {
                snapshot.cia402State = QString::fromStdString(
                    cia402::stateName(cia402::decodeState(snapshot.statusWord)));
            } else {
                snapshot.cia402State = tr("不支持/读取失败");
            }
            snapshot.controlValid = readSdo(slave, 0x6040, 0, snapshot.controlWord);
            snapshot.modeCommandValid = readSdo(slave, 0x6060, 0, snapshot.modeCommand);
            snapshot.modeDisplayValid = readSdo(slave, 0x6061, 0, snapshot.modeDisplay);
            if (snapshot.modeDisplayValid) {
                snapshot.modeText = QString::fromStdString(cia402::modeName(snapshot.modeDisplay));
            } else if (snapshot.modeCommandValid) {
                snapshot.modeText = QString::fromStdString(cia402::modeName(snapshot.modeCommand));
            }
            snapshot.positionValid = readSdo(slave, 0x6064, 0, snapshot.position);
            snapshot.velocityValid = readSdo(slave, 0x606c, 0, snapshot.velocity);
            snapshot.torqueValid = readSdo(slave, 0x6077, 0, snapshot.torque);
            snapshot.errorCodeValid = readSdo(slave, 0x603f, 0, snapshot.errorCode);
            snapshot.errorRegisterValid = readSdo(slave, 0x1001, 0, snapshot.errorRegister);
        } else {
            snapshot.cia402State = tr("从站不支持 CoE");
        }
        result.append(snapshot);
    }
    emit snapshotsReady(result);
}

void EthercatWorker::closeInterface()
{
    if (pollTimer_ != nullptr) pollTimer_->stop();
    demo_ = false;
    if (initialized_) {
        ecx_close(&context_);
        initialized_ = false;
    }
}

void EthercatWorker::ensurePollTimer()
{
    if (pollTimer_ != nullptr) return;
    pollTimer_ = new QTimer(this);
    pollTimer_->setInterval(pollIntervalMs_);
    connect(pollTimer_, &QTimer::timeout, this, &EthercatWorker::poll);
}

void EthercatWorker::scanDemo()
{
    demo_ = true;
    demoTick_ = 0;
    SlaveInfoList slaves;
    slaves.append({1, tr("演示伺服驱动器 A"), 0x000000fbU, 0x61500000U, 0x00010000U,
                   QStringLiteral("PRE-OP"), true, true});
    slaves.append({2, tr("演示伺服驱动器 B"), 0x0000009aU, 0x00030924U, 0x00020001U,
                   QStringLiteral("PRE-OP"), true, true});
    emit scanCompleted(slaves);
    emit connectionChanged(true, tr("演示模式：模拟 2 个 CiA402 从站"));
    pollTimer_->start();
    pollDemo();
}

void EthercatWorker::pollDemo()
{
    ++demoTick_;
    DriveSnapshotList result;
    for (int slave = 1; slave <= 2; ++slave) {
        DriveSnapshot s;
        s.slave = slave;
        s.ethercatState = QStringLiteral("OP");
        s.statusValid = true;
        s.statusWord = (slave == 2 && demoTick_ % 24 >= 20) ? 0x0008U : 0x0027U;
        s.cia402State = QString::fromStdString(cia402::stateName(cia402::decodeState(s.statusWord)));
        s.controlValid = true;
        s.controlWord = 0x000fU;
        s.modeCommandValid = s.modeDisplayValid = true;
        s.modeCommand = s.modeDisplay = 8;
        s.modeText = QString::fromStdString(cia402::modeName(8));
        s.positionValid = s.velocityValid = s.torqueValid = true;
        const double phase = (demoTick_ + slave * 3) * 0.22;
        s.position = static_cast<qint32>(std::sin(phase) * 100000.0);
        s.velocity = static_cast<qint32>(std::cos(phase) * 2200.0);
        s.torque = static_cast<qint16>(std::sin(phase) * 350.0);
        s.errorCodeValid = s.errorRegisterValid = true;
        s.errorCode = (s.statusWord == 0x0008U) ? 0x2310U : 0U;
        s.errorRegister = (s.statusWord == 0x0008U) ? 0x01U : 0U;
        result.append(s);
    }
    emit snapshotsReady(result);
}

QString EthercatWorker::ethercatStateName(const std::uint16_t state)
{
    QString name;
    switch (state & 0x000fU) {
    case EC_STATE_NONE: name = QStringLiteral("NONE"); break;
    case EC_STATE_INIT: name = QStringLiteral("INIT"); break;
    case EC_STATE_PRE_OP: name = QStringLiteral("PRE-OP"); break;
    case EC_STATE_SAFE_OP: name = QStringLiteral("SAFE-OP"); break;
    case EC_STATE_OPERATIONAL: name = QStringLiteral("OP"); break;
    default: name = QStringLiteral("0x%1").arg(state, 2, 16, QLatin1Char('0')); break;
    }
    if ((state & EC_STATE_ERROR) != 0) name += QStringLiteral(" + ERROR");
    return name;
}
