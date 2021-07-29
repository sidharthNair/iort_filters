#ifndef iort_filters_MQTTOverlay_DIALOG_HPP
#define iort_filters_MQTTOverlay_DIALOG_HPP

#include <insitu/filter.hpp>

namespace iort_filters
{

    class MQTTOverlayDialog : public insitu::FilterDialog
    {
        Q_OBJECT
    private:
        QPushButton *okButton;
        QPushButton *cancelButton;
        QLineEdit *text1;
        QGridLayout *layout;
        QLabel *desc1;
        QLabel *desc2;
        QListWidget *list;
        QCheckBox *cb;
        std::string uuid = "";
        std::vector<std::string> queries;

    public Q_SLOTS:

        void onOK(void);

    public:
        MQTTOverlayDialog(insitu::Filter *parent_);

        void updateList(void);

        std::vector<std::string> getQueries()
        {
            return queries;
        }
    };

} // end namespace iort_filters

#endif // end iort_filters_MQTTOverlay_DIALOG_HPP
