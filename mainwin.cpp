#include "mainwin.h"

Mainwin::Mainwin() : button {new Gtk::Button{"Hello, World"}
{
   add(*button);
   button->show();
   button->signal_clicked().connect([this] {this->on_button_click();};
   }
Mainwin::~Mainwin(){}

void Mainwin::on_button_click(){
   std::cout << "Hello World!" << std::endl;
}
