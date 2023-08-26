#pragma once

#include <string>
#include "Types.h"

#include "ShapeOp/Solver.h"

#include <fstream>
#include <iostream>

class CellExporter
{
public:
    static void writePositionsToFile(std::string filePath, ShapeOp::Matrix3X& positions)
    {
        std::ofstream outputFile(filePath);

        if (outputFile.is_open())
        {
            for (int i = 0; i < positions.cols(); i++)
            {
                outputFile << positions(0, i) << ",";
                outputFile << positions(1, i) << ",";
                outputFile << positions(2, i) << std::endl;
            }
            // Close the file
            outputFile.close();
            std::cout << "Numbers exported to " << filePath << " successfully." << std::endl;
        }
        else
        {
            std::cerr << "Unable to open the file " << filePath << " for writing." << std::endl;
        }
    }

    static void writeMeshToFile(std::string filePath, const Mesh& mesh)
    {
        std::ofstream outputFile(filePath);

        if (outputFile.is_open())
        {
            for (int i = 0; i < mesh.vertices.size(); i++)
            {
                outputFile << "v,";
                outputFile << mesh.vertices[i].x << ",";
                outputFile << mesh.vertices[i].y << ",";
                outputFile << mesh.vertices[i].z << std::endl;
            }

            for (int i = 0; i < mesh.normals.size(); i++)
            {
                outputFile << "vn,";
                outputFile << mesh.normals[i].x << ",";
                outputFile << mesh.normals[i].y << ",";
                outputFile << mesh.normals[i].z << std::endl;
            }

            for (int i = 0; i < mesh.faces.size(); i++)
            {
                outputFile << "f,";
                outputFile << mesh.faces[i].i0 << ",";
                outputFile << mesh.faces[i].i1 << ",";
                outputFile << mesh.faces[i].i2 << std::endl;
            }

            // Close the file
            outputFile.close();
            std::cout << "Numbers exported to " << filePath << " successfully." << std::endl;
        }
        else
        {
            std::cerr << "Unable to open the file " << filePath << " for writing." << std::endl;
        }
    }

private:

};
