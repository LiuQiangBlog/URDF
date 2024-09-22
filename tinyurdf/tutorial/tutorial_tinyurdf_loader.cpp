//
// Created by LiuQiang on 2024/9/22.
//
#include <TinyURDFParser.hpp>
#include <string>

int main()
{
    TinyURDFParser parser = TinyURDFParser::fromFile(
        "D:\\VideoNotes\\Handbook_For_Robot_Beginners\\Code\\Thirdparty\\tinyurdf\\tutorial\\simple_robot.urdf");
    auto links = parser.getLinks();
    return 0;
}