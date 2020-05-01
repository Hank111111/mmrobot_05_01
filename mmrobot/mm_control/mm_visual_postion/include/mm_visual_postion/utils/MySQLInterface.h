#ifndef __MYSQL_INTERFACE_H__
#define __MYSQL_INTERFACE_H__
/*
  Include directly the different
  headers from cppconn/ and mysql_driver.h + mysql_util.h
  (and mysql_connection.h). This will reduce your build time!
*/
#include "mysql_connection.h"

#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
class MySQLInterface{
  sql::Driver *driver;


public:
    sql::Connection *con;
    MySQLInterface(std::string adress_and_port, std::string username, std::string password, std::string schema){
        //std::cout<<"adress and port: "<<adress_and_port <<" username: "<< username << " password: "<< password<< " schema: "<<schema<<std::endl;
        driver = get_driver_instance();
        con = driver->connect(adress_and_port, username, password);
        con->setSchema(schema);
    }
    ~MySQLInterface(){
        delete con;
    }

};
#endif