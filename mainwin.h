#include <gtkmm>

class Mainwin : public Gtk::Window {
   public:
      Mainwin();
      virtual ~Maintain();
   protected:
      Gtk::Button button;
      void on_button_click();
};

