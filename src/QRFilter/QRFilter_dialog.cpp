#include <QRFilter/QRFilter_dialog.hpp>

namespace iort_filters
{
    QRFilterDialog::QRFilterDialog(insitu::Filter *parent_) : FilterDialog(parent_)
    {
        okButton = new QPushButton(tr("OK"));
        okButton->setDefault(true);
        cancelButton = new QPushButton(tr("Cancel"));
        desc = new QLabel(tr("Data Member List"));
        list = new QListWidget();
        list->setSpacing(5);
        list->setDragEnabled(true);
        list->viewport()->setAcceptDrops(true);
        list->setDropIndicatorShown(true);
        list->setDragDropMode(QAbstractItemView::InternalMove);
        cb = new QCheckBox("Generate bar view (0-4095) for integer members");

        layout = new QGridLayout();
        layout->addWidget(desc, 0, 0, 1, 2);
        layout->addWidget(list, 1, 0, 1, 2);
        layout->addWidget(cb, 2, 0, 1, 2);
        layout->addWidget(cancelButton, 3, 0);
        layout->addWidget(okButton, 3, 1);

        setLayout(layout);

        QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
        QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
    }

    void QRFilterDialog::onOK(void)
    {
        Json::Value &settings = parent->getSettingsValue();
        queries.clear();
        int bars = 0;
        for (int i = 0; i < list->count(); i++)
        {
            QListWidgetItem *item = list->item(i);
            if (item->checkState())
            {
                queries.push_back(item->text().toStdString());
                if (settings["data"].get(item->text().toStdString(), 0).isInt())
                {
                    bars++;
                }
            }
        }
        settings["bars"] = bars;
        settings["generate_bars"] = (bool)cb->checkState();
        settings["update_list"] = true;
        accept();
    }

    void QRFilterDialog::updateList(void)
    {
        Json::Value &settings = parent->getSettingsValue();
        std::vector<std::string> members = settings["data"].getMemberNames();
        int bars = 0;
        for (int i = 0; i < members.size(); i++)
        {
            queries.push_back(members[i]);
            if (settings["data"].get(members[i], 0).isInt())
            {
                bars++;
            }
            QListWidgetItem *item = new QListWidgetItem(members[i].c_str(), list);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable); // set checkable flag
            item->setCheckState(Qt::Checked);                        // AND initialize check state
            list->addItem(item);
        }
        settings["bars"] = bars;
    }

} // end namespace iort_filters
