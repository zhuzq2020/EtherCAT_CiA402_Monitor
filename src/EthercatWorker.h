#pragma once

#include "Models.h"

#include <QObject>

#include <cstdint>
#include <type_traits>

extern "C" {
#include <soem/soem.h>
}

class QTimer;

class EthercatWorker final : public QObject {
    Q_OBJECT

public:
    explicit EthercatWorker(QObject *parent = nullptr);
    ~EthercatWorker() override;

public slots:
    void discoverAdapters();
    void scan(const QString &interfaceName);
    void stop();
    void setPollInterval(int milliseconds);

signals:
    void adaptersDiscovered(const AdapterInfoList &adapters);
    void scanCompleted(const SlaveInfoList &slaves);
    void snapshotsReady(const DriveSnapshotList &snapshots);
    void connectionChanged(bool connected, const QString &message);
    void logMessage(const QString &message);

private slots:
    void poll();

private:
    template<typename T>
    bool readSdo(int slave, std::uint16_t index, std::uint8_t subIndex, T &value)
    {
        static_assert(std::is_integral_v<T>, "SDO helper supports integral values only");
        T raw{};
        int size = static_cast<int>(sizeof(T));
        const int wkc = ecx_SDOread(&context_, static_cast<std::uint16_t>(slave), index,
                                    subIndex, FALSE, &size, &raw, kSdoTimeoutUs);
        if (wkc <= 0 || size != static_cast<int>(sizeof(T))) return false;

        if constexpr (sizeof(T) == 1) {
            value = raw;
        } else if constexpr (sizeof(T) == 2) {
            const auto host = etohs(static_cast<std::uint16_t>(raw));
            value = static_cast<T>(host);
        } else if constexpr (sizeof(T) == 4) {
            const auto host = etohl(static_cast<std::uint32_t>(raw));
            value = static_cast<T>(host);
        }
        return true;
    }

    void closeInterface();
    void ensurePollTimer();
    void scanDemo();
    void pollDemo();
    static QString ethercatStateName(std::uint16_t state);

    static constexpr int kSdoTimeoutUs = 50000;
    static constexpr const char *kDemoInterface = "__demo__";

    ecx_contextt context_{};
    QTimer *pollTimer_ = nullptr;
    int pollIntervalMs_ = 500;
    bool initialized_ = false;
    bool demo_ = false;
    int demoTick_ = 0;
};
