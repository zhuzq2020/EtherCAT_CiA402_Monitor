#include "MainWindow.h"

#include <QApplication>
#include <QPalette>
#include <QStyleFactory>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("EtherCAT CiA402 Monitor"));
    QApplication::setOrganizationName(QStringLiteral("admin1"));
    QApplication::setStyle(QStyleFactory::create(QStringLiteral("Fusion")));

    QPalette palette;
    palette.setColor(QPalette::Window, QColor(QStringLiteral("#f6f8fa")));
    palette.setColor(QPalette::WindowText, QColor(QStringLiteral("#1f2328")));
    palette.setColor(QPalette::Base, Qt::white);
    palette.setColor(QPalette::AlternateBase, QColor(QStringLiteral("#f6f8fa")));
    palette.setColor(QPalette::Text, QColor(QStringLiteral("#1f2328")));
    palette.setColor(QPalette::Button, QColor(QStringLiteral("#f6f8fa")));
    palette.setColor(QPalette::ButtonText, QColor(QStringLiteral("#1f2328")));
    palette.setColor(QPalette::Highlight, QColor(QStringLiteral("#0969da")));
    palette.setColor(QPalette::HighlightedText, Qt::white);
    app.setPalette(palette);

    MainWindow window;
    window.show();
    return app.exec();
}
