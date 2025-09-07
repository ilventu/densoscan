#include "densoscan.h"
#include "densoscan_wx.h"

#include <wx/wx.h>

class MyApp : public wxApp {
public:
    virtual bool OnInit() override {
        DensoScanFrame* frame = new DensoScanFrame("DensoScan");
        frame->Show(true);
        return true;
    }
};

wxIMPLEMENT_APP(MyApp);
