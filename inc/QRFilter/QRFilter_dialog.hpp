#ifndef iort_filters_QRFilter_DIALOG_HPP
#define iort_filters_QRFilter_DIALOG_HPP

#include <insitu/filter.hpp>

namespace iort_filters
{

    class QRFilterDialog : public insitu::FilterDialog
    {
        Q_OBJECT
    private:
        QPushButton *okButton;
        QPushButton *cancelButton;
        QGridLayout *layout;
        QLabel *desc;
        QListWidget *list;
        QCheckBox *cb;
        std::vector<std::string> queries;

    public Q_SLOTS:

        void onOK(void);

    public:
        QRFilterDialog(insitu::Filter *parent_);

        void updateList(void);

        std::vector<std::string> getQueries()
        {
            return queries;
        }
    };

} // end namespace iort_filters

#endif // end iort_filters_QRFilter_DIALOG_HPP
