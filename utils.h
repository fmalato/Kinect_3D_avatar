#include <vector>
#include <algorithm>
#include <iostream>

#include <GLFW/glfw3.h>
#include <fstream>
#include <sstream>

void drawGrid() {

    glLineWidth(3);
    glColor3f(1,1,1);
    glBegin(GL_LINE_LOOP);
    glVertex3f( 0,-0.001, 0);
    glVertex3f( 0,-0.001,10);
    glVertex3f(10,-0.001,10);
    glVertex3f(10,-0.001, 0);
    glEnd();

    glBegin(GL_LINES);
    for(int i=0;i<=10;i++) {
        if (i==0) { glColor3f(.6,.3,.3); } else { glColor3f(.25,.25,.25); };
        glVertex3f(i,0,0);
        glVertex3f(i,0,10);
        if (i==0) { glColor3f(.3,.3,.6); } else { glColor3f(.25,.25,.25); };
        glVertex3f(0,0,i);
        glVertex3f(10,0,i);
    };
    glEnd();

}

std::vector<std::vector<double>> getJointPositions(std::string fileName) {

    std::fstream fin;
    fin.open(fileName, std::ios::in);
    std::vector<std::string> row;
    std::string line, word, temp;
    std::vector<std::vector<double>> positions;

    while(fin >> temp) {

        row.clear();
        getline(fin, line);
        std::stringstream s(line);
        while(getline(s, word, ' ')) {

            row.push_back(word);

        }

        for(int i = 0; i < row.size() / 3; i += 3) {

            positions.push_back({std::stod(row[i]), std::stod(row[i + 1]), std::stod(row[i + 2])});

        }

    }

    return positions;

}