#pragma once

#include "Models.h"

#include <QMainWindow>
#include <QThread>

class EthercatWorker;
class QComboBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void updateAdapters(const AdapterInfoList &adapters);
    void updateSlaveList(const SlaveInfoList &slaves);
    void updateSnapshots(const DriveSnapshotList &snapshots);
    void updateConnection(bool connected, const QString &message);
    void appendLog(const QString &message);

private:
    void buildUi();
    void registerMetaTypes();
    static QString hexValue(quint64 value, int width);
    static QString validNumber(bool valid, qint64 value);

    QThread workerThread_;
    EthercatWorker *worker_ = nullptr;
    QComboBox *adapterCombo_ = nullptr;
    QPushButton *refreshButton_ = nullptr;
    QPushButton *scanButton_ = nullptr;
    QPushButton *stopButton_ = nullptr;
    QSpinBox *pollInterval_ = nullptr;
    QLabel *statusIndicator_ = nullptr;
    QLabel *statusLabel_ = nullptr;
    QTableWidget *table_ = nullptr;
    QPlainTextEdit *log_ = nullptr;
};

