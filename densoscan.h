#ifndef DENSOSCAN_H
#define DENSOSCAN_H

#include <QMainWindow>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QSocketNotifier>
#include <QEvent>
#include <QtCharts>
#include <QLineSeries>

#include "scanner.h"

QT_BEGIN_NAMESPACE
namespace Ui { class DensoScan; }
QT_END_NAMESPACE

class QProfile : public QObject, public Profile
{
public:
    QString name;
    bool valid = false;

    QJsonObject toJson ()
    {
        QJsonArray jexpected, jmeasured;
        std::copy ( expected.begin(), expected.end(), std::back_inserter(jexpected));
        std::copy ( measured.begin(), measured.end(), std::back_inserter(jmeasured));

        QJsonObject jobject;
        jobject["name"] = name;
        jobject["expected"] = jexpected;
        jobject["measuerd"] = jmeasured;

        return jobject;
    }

    void fromJson ( const QJsonObject &jobject )
    {
        QJsonArray jexpected, jmeasured;

        name = jobject["name"].toString();
        jexpected = jobject["expected"].toArray();
        jmeasured = jobject["measuerd"].toArray();

        measured.empty();
        expected.empty();
        for (const auto& element : jexpected )
            expected.push_back( element.toDouble() );

        for (const auto& element : jmeasured )
            measured.push_back( element.toDouble() );

        std::sort(expected.begin(), expected.end(), std::less<double>());
        std::sort(measured.begin(), measured.end(), std::less<double>());
    }
};

class QProfiles : public QObjectList
{
public:
    QProfile *at ( int i )
    {
        return (QProfile *)QObjectList::at(i);
    }

    QJsonObject toJson ()
    {
        QJsonArray jprofiles;
        for (const auto& element : *this )
            jprofiles.push_back( ((QProfile *)element)->toJson( ) );

        QJsonObject jobject;

        jobject["profiles"] = jprofiles;

        return jobject;
    }

    void fromJson ( const QJsonObject &jobject )
    {
        QJsonArray jprofiles;

        jprofiles = jobject["profiles"].toArray();

        for (const auto& element : jprofiles )
        {
            QProfile *p = new QProfile;
            p->fromJson(element.toObject());
            push_back( p );
        }
    }
};

class DScanner : public Scanner
{
    void onNewScan ( Scan &image, int frameNumber );
    void onScanCompleted ();
    void onPreviewCompleted ( const Scan &preview );
    void onProgressUpdate ( const std::string text, int percent );
    void onError ( const std::string error );
};

const QEvent::Type EVENT_PREVIEW_COMPLETED  = static_cast<QEvent::Type>(QEvent::User + 1);
const QEvent::Type EVENT_SCAN_COMPLETED     = static_cast<QEvent::Type>(QEvent::User + 2);
const QEvent::Type EVENT_PROGRESS_UPDATE    = static_cast<QEvent::Type>(QEvent::User + 3);
const QEvent::Type EVENT_ERROR              = static_cast<QEvent::Type>(QEvent::User + 4);
const QEvent::Type EVENT_NEW_SCAN           = static_cast<QEvent::Type>(QEvent::User + 5);

class NewScanEvent : public QEvent
{
public:
    Scan image;
    int frameNumber;
    NewScanEvent ( const Scan &image, int frameNumber ): QEvent(EVENT_NEW_SCAN), image ( image ), frameNumber (frameNumber) {}
};

class ScanCompletedEvent : public QEvent
{
public:
    Scan preview;
    ScanCompletedEvent (  ): QEvent(EVENT_SCAN_COMPLETED) {}
};

class PreviewCompletedEvent : public QEvent
{
public:
    Scan preview;
    PreviewCompletedEvent ( const Scan &preview ): QEvent(EVENT_PREVIEW_COMPLETED), preview (preview) {}
};

class ProgressUpdateEvent : public QEvent
{
public:
    std::string text;
    int percent;
    ProgressUpdateEvent ( const std::string text, int percent ): QEvent(EVENT_PROGRESS_UPDATE), text (text), percent (percent) {}
};

class ErrorEvent : public QEvent
{
public:
    std::string error;
    ErrorEvent ( const std::string &error ): QEvent(EVENT_ERROR), error ( error ) {}
};

class DensoScan : public QMainWindow
{
    Q_OBJECT

    QProfiles profiles;
    DScanner scanner;

    std::vector<std::string> filenames;

private:
  static DensoScan* instance;

public:
    DensoScan(QWidget *parent = nullptr);
    ~DensoScan();

    static DensoScan* getInstance ()
    {
      return instance;
    }

    void scan ( const std::string &name, std::vector <Box> boxes, Profile * );
    void startScan ();

    void loadDeviceSettings ();
    void saveDeviceSettings ();
    void loadOptions ();
    void saveOptions ();
    void loadProfiles ();

    void enableOptions ( bool enabled );
    void updateDeviceOptions();

    void drawLogo ();

    QChartView *chartView = nullptr;
    void drawChart ();

protected:
    void customEvent(QEvent *event); // This overrides QObject::customEvent()

    void onNewScan ( Scan &image, int frameNumber );
    void onScanCompleted ();
    void onPreviewCompleted ( const Scan &preview );
    void onProgressUpdate ( const std::string text, int percent );
    void onError ( const std::string &error );

private slots:
    void on_buttonRefresh_clicked();

    void on_comboDevice_currentIndexChanged(int index);

    void on_pushScan_clicked();

    void on_pushCancel_clicked();

    void on_toolSelectFolder_clicked();

    void on_outputType_currentIndexChanged(int index);

    void on_comboProfile_currentIndexChanged(int index);

    void on_pushOptions_clicked();

private:
    Ui::DensoScan *ui;
};

#endif // DENSOSCAN_H
