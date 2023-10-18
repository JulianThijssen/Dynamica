#pragma once

#include "ObjLoader.h"
#include "CellExporter.h"

class Recorder
{
public:
    void addMeshObservation(Mesh* mesh)
    {
        _meshes.push_back(mesh);
    }

    void start()
    {
        _frame = 0;
        _recording = true;
    }

    void recordFrame(Scene& scene)
    {
        if (!_recording)
            return;

        for (int i = 0; i < scene.cells.size(); i++)
        {
            Mesh& mesh = scene.cells[i];
            std::string filePath = "Frames/cell_" + std::to_string(i) + "_" + std::to_string(_frame) + ".csv";
            CellExporter::writeMeshToFile(filePath, mesh);
        }
        _frame++;
    }

    void stop()
    {
        _recording = false;
    }

private:
    std::vector<Mesh*> _meshes;

    bool _recording = false;
    int _frame = 0;
};
