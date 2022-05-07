/*********************************************************************** 
 * Derechos reservados                                                 *
 * Autores: Leticia Oyuki Rojas Perez, Jose Martinez Carranza          * 
 * Versión: 1.0                                                        *
 * Última actualización:  05/06/2018                                   *
 *                                                                     *
 * Ejemplo de Interfaz de control manual                               *
 *                                                                     *     
 ***********************************************************************/
 
#include "keyboard.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "keyboard");
  QApplication app(argc, argv);  
    
  KeyPress window;
  
  window.resize(300, 100);
  window.setWindowTitle("Keyboard Bebop2");
  window.show();
  
  return app.exec();
}
