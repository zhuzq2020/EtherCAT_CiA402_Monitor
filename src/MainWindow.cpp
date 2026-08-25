#include "MainWindow.h"

#include "EthercatWorker.h"

#include <QComboBox>
#include <QColor>
#include <QBrush>
#include <QDateTime>
#include <QHeaderView>
#include <QLabel>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QTableWidget>
#include <QVBoxLayout>

namespace {
enum Column {
    SlaveColumn,
    NameColumn,
    EthercatColumn,
    Cia402Column,
    StatusWordColumn,
    ControlWordColumn,
    ModeColumn,
    PositionColumn,
    VelocityColumn,
    TorqueColumn,
    ErrorColumn,
    ColumnCount
};
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    registerMetaTypes();
    buildUi();

    worker_ = new EthercatWorker;
    worker_->moveToThread(&workerThread_);
    connect(&workerThread_, &QThread::finished, worker_, &QObject::deleteLater);
    connect(worker_, &EthercatWorker::adaptersDiscovered, this, &MainWindow::updateAdapters);
    connect(worker_, &EthercatWorker::scanCompleted, this, &MainWindow::updateSlaveList);
    connect(worker_, &EthercatWorker::snapshotsReady, this, &MainWindow::updateSnapshots);
    connect(worker_, &EthercatWorker::connectionChanged, this, &MainWindow::updateConnection);
    connect(worker_, &EthercatWorker::logMessage, this, &MainWindow::appendLog);
    workerThread_.start();

    connect(refreshButton_, &QPushButton::clicked, worker_, &EthercatWorker::discoverAdapters);
    connect(scanButton_, &QPushButton::clicked, this, [this] {
        const QString interfaceName = adapterCombo_->currentData().toString();
        if (interfaceName.isEmpty()) return;
        table_->setRowCount(0);
        scanButton_->setEnabled(false);
        QMetaObject::invokeMethod(worker_, "scan", Qt::QueuedConnection, Q_ARG(QString, interfaceName));
    });
    connect(stopButton_, &QPushButton::clicked, worker_, &EthercatWorker::stop);
    connect(pollInterval_, qOverload<int>(&QSpinBox::valueChanged),
            worker_, &EthercatWorker::setPollInterval);

    QMetaObject::invokeMethod(worker_, "discoverAdapters", Qt::QueuedConnection);
}

MainWindow::~MainWindow()
{
    if (workerThread_.isRunning()) {
        QMetaObject::invokeMethod(worker_, "stop", Qt::BlockingQueuedConnection);
        workerThread_.quit();
        workerThread_.wait();
    }
}

void MainWindow::buildUi()
{
    setWindowTitle(tr("EtherCAT CiA402 状态机在线监控"));
    resize(1500, 760);

    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);
    auto *toolbar = new QHBoxLayout;

    toolbar->addWidget(new QLabel(tr("网络接口:"), this));
    adapterCombo_ = new QComboBox(this);
    adapterCombo_->setMinimumWidth(340);
    toolbar->addWidget(adapterCombo_);
    refreshButton_ = new QPushButton(tr("刷新网卡"), this);
    scanButton_ = new QPushButton(tr("扫描并监控"), this);
    stopButton_ = new QPushButton(tr("停止"), this);
    toolbar->addWidget(refreshButton_);
    toolbar->addWidget(scanButton_);
    toolbar->addWidget(stopButton_);
    toolbar->addSpacing(16);
    toolbar->addWidget(new QLabel(tr("刷新周期:"), this));
    pollInterval_ = new QSpinBox(this);
    pollInterval_->setRange(100, 10000);
    pollInterval_->setValue(500);
    pollInterval_->setSuffix(tr(" ms"));
    toolbar->addWidget(pollInterval_);
    toolbar->addStretch();

    statusIndicator_ = new QLabel(this);
    statusIndicator_->setFixedSize(12, 12);
    statusIndicator_->setStyleSheet(QStringLiteral("background:#8b949e;border-radius:6px;"));
    statusLabel_ = new QLabel(tr("未连接"), this);
    toolbar->addWidget(statusIndicator_);
    toolbar->addWidget(statusLabel_);
    root->addLayout(toolbar);

    auto *splitter = new QSplitter(Qt::Vertical, this);
    table_ = new QTableWidget(0, ColumnCount, splitter);
    table_->setHorizontalHeaderLabels({
        tr("从站"), tr("名称 / 设备"), tr("EtherCAT"), tr("CiA402 状态"),
        tr("状态字 6041h"), tr("控制字 6040h"), tr("运行模式 6061h"),
        tr("位置 6064h"), tr("速度 606Ch"), tr("转矩 6077h"), tr("错误 603Fh / 1001h")
    });
    table_->setAlternatingRowColors(true);
    table_->setStyleSheet(QStringLiteral(
        "QTableWidget { background-color:#ffffff; alternate-background-color:#f6f8fa;"
        " color:#1f2328; gridline-color:#d0d7de; selection-background-color:#0969da;"
        " selection-color:#ffffff; }"
        "QHeaderView::section { background-color:#eaeef2; color:#1f2328;"
        " border:0; border-right:1px solid #d0d7de; border-bottom:1px solid #d0d7de;"
        " padding:6px; font-weight:600; }"));
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->verticalHeader()->setVisible(false);
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    table_->horizontalHeader()->setSectionResizeMode(NameColumn, QHeaderView::Stretch);
    table_->horizontalHeader()->setSectionResizeMode(Cia402Column, QHeaderView::Stretch);

    log_ = new QPlainTextEdit(splitter);
    log_->setReadOnly(true);
    log_->setStyleSheet(QStringLiteral(
        "QPlainTextEdit { background-color:#ffffff; color:#1f2328; border:1px solid #d0d7de; }"));
    log_->setMaximumBlockCount(1000);
    log_->setPlaceholderText(tr("运行日志"));
    splitter->addWidget(table_);
    splitter->addWidget(log_);
    splitter->setStretchFactor(0, 5);
    splitter->setStretchFactor(1, 1);
    root->addWidget(splitter);

    setCentralWidget(central);
    statusBar()->showMessage(tr("只读模式：不会写入控制字或使能电机"));
}

void MainWindow::registerMetaTypes()
{
    qRegisterMetaType<AdapterInfoList>("AdapterInfoList");
    qRegisterMetaType<SlaveInfoList>("SlaveInfoList");
    qRegisterMetaType<DriveSnapshotList>("DriveSnapshotList");
}

void MainWindow::updateAdapters(const AdapterInfoList &adapters)
{
    const QString previous = adapterCombo_->currentData().toString();
    adapterCombo_->clear();
    for (const AdapterInfo &adapter : adapters) {
        const QString label = adapter.name == QStringLiteral("__demo__")
            ? adapter.description
            : QStringLiteral("%1 — %2").arg(adapter.name, adapter.description);
        adapterCombo_->addItem(label, adapter.name);
    }
    const int previousIndex = adapterCombo_->findData(previous);
    if (previousIndex >= 0) adapterCombo_->setCurrentIndex(previousIndex);
    scanButton_->setEnabled(adapterCombo_->count() > 0);
    appendLog(tr("发现 %1 个可用项（含演示模式）").arg(adapters.size()));
}

void MainWindow::updateSlaveList(const SlaveInfoList &slaves)
{
    table_->setRowCount(slaves.size());
    for (int row = 0; row < slaves.size(); ++row) {
        const SlaveInfo &slave = slaves.at(row);
        auto set = [this, row](int column, const QString &text) {
            table_->setItem(row, column, new QTableWidgetItem(text));
        };
        set(SlaveColumn, QString::number(slave.index));
        set(NameColumn, slave.name);
        table_->item(row, NameColumn)->setToolTip(
            tr("Vendor: %1\nProduct: %2\nRevision: %3\nCoE: %4\nDC: %5")
                .arg(hexValue(slave.vendorId, 8), hexValue(slave.productCode, 8), hexValue(slave.revision, 8),
                     slave.coeSupported ? tr("是") : tr("否"), slave.dcSupported ? tr("是") : tr("否")));
        set(EthercatColumn, slave.ethercatState);
        for (int column = Cia402Column; column < ColumnCount; ++column) set(column, QStringLiteral("—"));
    }
    scanButton_->setEnabled(true);
    appendLog(tr("扫描完成：%1 个从站").arg(slaves.size()));
}

void MainWindow::updateSnapshots(const DriveSnapshotList &snapshots)
{
    for (const DriveSnapshot &snapshot : snapshots) {
        const int row = snapshot.slave - 1;
        if (row < 0 || row >= table_->rowCount()) continue;
        table_->item(row, EthercatColumn)->setText(snapshot.ethercatState);
        table_->item(row, Cia402Column)->setText(snapshot.cia402State);
        table_->item(row, StatusWordColumn)->setText(
            snapshot.statusValid ? hexValue(snapshot.statusWord, 4) : QStringLiteral("—"));
        table_->item(row, ControlWordColumn)->setText(
            snapshot.controlValid ? hexValue(snapshot.controlWord, 4) : QStringLiteral("—"));
        table_->item(row, ModeColumn)->setText(snapshot.modeText.isEmpty() ? QStringLiteral("—") : snapshot.modeText);
        table_->item(row, PositionColumn)->setText(validNumber(snapshot.positionValid, snapshot.position));
        table_->item(row, VelocityColumn)->setText(validNumber(snapshot.velocityValid, snapshot.velocity));
        table_->item(row, TorqueColumn)->setText(validNumber(snapshot.torqueValid, snapshot.torque));

        QString error = QStringLiteral("—");
        if (snapshot.errorCodeValid || snapshot.errorRegisterValid) {
            error = QStringLiteral("%1 / %2")
                .arg(snapshot.errorCodeValid ? hexValue(snapshot.errorCode, 4) : QStringLiteral("—"),
                     snapshot.errorRegisterValid ? hexValue(snapshot.errorRegister, 2) : QStringLiteral("—"));
        }
        table_->item(row, ErrorColumn)->setText(error);

        const bool fault = snapshot.statusValid &&
            (snapshot.cia402State == QStringLiteral("Fault") ||
             snapshot.cia402State == QStringLiteral("Fault reaction active"));
        for (int column = 0; column < ColumnCount; ++column) {
            QTableWidgetItem *item = table_->item(row, column);
            item->setBackground(fault ? QBrush(QColor(QStringLiteral("#ffebe9"))) : QBrush());
            item->setForeground(fault ? QBrush(QColor(QStringLiteral("#82071e"))) : QBrush());
        }
    }
}

void MainWindow::updateConnection(const bool connected, const QString &message)
{
    statusIndicator_->setStyleSheet(QStringLiteral("background:%1;border-radius:6px;")
        .arg(connected ? QStringLiteral("#2da44e") : QStringLiteral("#cf222e")));
    statusLabel_->setText(message);
    scanButton_->setEnabled(true);
    appendLog(message);
}

void MainWindow::appendLog(const QString &message)
{
    log_->appendPlainText(QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz")), message));
}

QString MainWindow::hexValue(const quint64 value, const int width)
{
    return QStringLiteral("0x%1").arg(value, width, 16, QLatin1Char('0')).toUpper();
}

QString MainWindow::validNumber(const bool valid, const qint64 value)
{
    return valid ? QString::number(value) : QStringLiteral("—");
}
