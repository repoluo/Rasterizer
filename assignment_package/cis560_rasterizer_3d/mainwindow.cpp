#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
#include <QFileDialog>
#include <QTextStream>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <iostream>
#include <QApplication>
#include <QKeyEvent>
#include <QImageWriter>
#include <QDebug>
#include <tiny_obj_loader.h>

//Poke around in this file if you want, but it's virtually uncommented!
//You won't need to modify anything in here to complete the assignment.

void MainWindow::keyPressEvent(QKeyEvent *e) {
    //The key shortcuts for the other menu commands were set in Qt's GUI
    //editor. This one was implemented as a key press event for illustration purposes.
    switch (e->key()) {
    case Qt::Key_Escape:
        on_actionQuit_Esc_triggered();
        break;

    // Translate or rotate camera based on operation mode
    case Qt::Key_Q:
        if (opMode == OperationMode::Translation) {
            rasterizer.getCamera().yDirMove(-1.5);
        } else if (opMode == OperationMode::Rotation) {
            rasterizer.getCamera().zDirRotationDegree(-85);
        }
        break;

    case Qt::Key_E:
        if (opMode == OperationMode::Translation) {
            rasterizer.getCamera().yDirMove(1.5);
        } else if (opMode == OperationMode::Rotation) {
            rasterizer.getCamera().zDirRotationDegree(85);
        }
        break;

    case Qt::Key_A:
        if (opMode == OperationMode::Translation) {
            rasterizer.getCamera().xDirMove(-1.5);
        } else if (opMode == OperationMode::Rotation) {
            rasterizer.getCamera().yDirRotationDegree(85);
        }
        break;

    case Qt::Key_D:
        if (opMode == OperationMode::Translation) {
            rasterizer.getCamera().xDirMove(1.5);
        } else if (opMode == OperationMode::Rotation) {
            rasterizer.getCamera().yDirRotationDegree(-85);
        }
        break;

    case Qt::Key_W:
        if (opMode == OperationMode::Translation) {
            rasterizer.getCamera().zDirMove(1.5);
        } else if (opMode == OperationMode::Rotation) {
            rasterizer.getCamera().xDirRotationDegree(85);
        }
        break;

    case Qt::Key_S:
        if (opMode == OperationMode::Translation) {
            rasterizer.getCamera().zDirMove(-1.5);
        } else if (opMode == OperationMode::Rotation) {
            rasterizer.getCamera().xDirRotationDegree(-85);
        }
        break;

    // Change operation mode
    case Qt::Key_R:
        opMode = OperationMode::Rotation;
        break;

    case Qt::Key_T:
        opMode = OperationMode::Translation;
        break;

    default:
        break;
    }

    rendered_image = rasterizer.renderScene();
    DisplayQImage(rendered_image);
}


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    rasterizer(std::vector<Polygon>())
{
    ui->setupUi(this);
    setFocusPolicy(Qt::StrongFocus);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::DisplayQImage(QImage &i)
{
    QPixmap pixmap(QPixmap::fromImage(i));
    graphics_scene.addPixmap(pixmap);
    graphics_scene.setSceneRect(pixmap.rect());
    ui->scene_display->setScene(&graphics_scene);
}

void MainWindow::on_actionLoad_Scene_triggered()
{
    std::vector<Polygon> polygons;

    QString filename = QFileDialog::getOpenFileName(0, QString("Load Scene File"), QDir::currentPath().append(QString("../..")), QString("*.json"));
    // in the case of quit to select file
    if (filename.length() == 0) {
        return;
    }
    int i = filename.length() - 1;
    while(QString::compare(filename.at(i), QChar('/')) != 0)
    {
        i--;
    }
    QString local_path = filename.left(i+1);

    QFile file(filename);
    if(!file.open(QIODevice::ReadOnly)){
        qWarning("Could not open the JSON file.");
        return;
    }
    QByteArray file_data = file.readAll();

    QJsonDocument jdoc(QJsonDocument::fromJson(file_data));
    //Read the mesh data in the file
    QJsonArray objects = jdoc.object()["objects"].toArray();
    for(int i = 0; i < objects.size(); i++)
    {
        std::vector<glm::vec4> vert_pos;
        std::vector<glm::vec3> vert_col;
        QJsonObject obj = objects[i].toObject();
        QString type = obj["type"].toString();
        //Custom Polygon case
        if(QString::compare(type, QString("custom")) == 0)
        {
            QString name = obj["name"].toString();
            QJsonArray pos = obj["vertexPos"].toArray();
            for(int j = 0; j < pos.size(); j++)
            {
                QJsonArray arr = pos[j].toArray();
                glm::vec4 p(arr[0].toDouble(), arr[1].toDouble(), arr[2].toDouble(), 1);
                vert_pos.push_back(p);
            }
            QJsonArray col = obj["vertexCol"].toArray();
            for(int j = 0; j < col.size(); j++)
            {
                QJsonArray arr = col[j].toArray();
                glm::vec3 c(arr[0].toDouble(), arr[1].toDouble(), arr[2].toDouble());
                vert_col.push_back(c);
            }
            Polygon p(name, vert_pos, vert_col);
            polygons.push_back(p);
        }
        //Regular Polygon case
        else if(QString::compare(type, QString("regular")) == 0)
        {
            QString name = obj["name"].toString();
            int sides = obj["sides"].toInt();
            QJsonArray colorA = obj["color"].toArray();
            glm::vec3 color(colorA[0].toDouble(), colorA[1].toDouble(), colorA[2].toDouble());
            QJsonArray posA = obj["pos"].toArray();
            glm::vec4 pos(posA[0].toDouble(), posA[1].toDouble(), posA[2].toDouble(),1);
            float rot = obj["rot"].toDouble();
            QJsonArray scaleA = obj["scale"].toArray();
            glm::vec4 scale(scaleA[0].toDouble(), scaleA[1].toDouble(), scaleA[2].toDouble(),1);
            Polygon p(name, sides, color, pos, rot, scale);
            polygons.push_back(p);
        }
        //OBJ file case
        else if(QString::compare(type, QString("obj")) == 0)
        {
            QString name = obj["name"].toString();
            QString filename = local_path;
            filename.append(obj["filename"].toString());
            Polygon p = LoadOBJ(filename, name);
            QString texPath = local_path;
            texPath.append(obj["texture"].toString());
            p.SetTexture(new QImage(texPath));
            if(obj.contains(QString("normalMap")))
            {
                p.SetNormalMap(new QImage(local_path.append(obj["normalMap"].toString())));
            }
            polygons.push_back(p);
        }
    }

    rasterizer = Rasterizer(polygons);
    hasLoadScene = true;
    rendered_image = rasterizer.renderScene();
    DisplayQImage(rendered_image);
}


Polygon MainWindow::LoadOBJ(const QString &file, const QString &polyName)
{
    Polygon p(polyName);
    QString filepath = file;
    std::vector<tinyobj::shape_t> shapes; std::vector<tinyobj::material_t> materials;
    std::string errors = tinyobj::LoadObj(shapes, materials, filepath.toStdString().c_str());
    std::cout << errors << std::endl;
    if(errors.size() == 0)
    {
        int min_idx = 0;
        //Read the information from the vector of shape_ts
        for(unsigned int i = 0; i < shapes.size(); i++)
        {
            std::vector<glm::vec4> pos, nor;
            std::vector<glm::vec2> uv;
            std::vector<float> &positions = shapes[i].mesh.positions;
            std::vector<float> &normals = shapes[i].mesh.normals;
            std::vector<float> &uvs = shapes[i].mesh.texcoords;
            for(unsigned int j = 0; j < positions.size()/3; j++)
            {
                pos.push_back(glm::vec4(positions[j*3], positions[j*3+1], positions[j*3+2],1));
            }
            for(unsigned int j = 0; j < normals.size()/3; j++)
            {
                nor.push_back(glm::vec4(normals[j*3], normals[j*3+1], normals[j*3+2],0));
            }
            for(unsigned int j = 0; j < uvs.size()/2; j++)
            {
                uv.push_back(glm::vec2(uvs[j*2], uvs[j*2+1]));
            }
            for(unsigned int j = 0; j < pos.size(); j++)
            {
                p.AddVertex(Vertex(pos[j], glm::vec3(255,255,255), nor[j], uv[j]));
            }

            std::vector<unsigned int> indices = shapes[i].mesh.indices;
            for(unsigned int j = 0; j < indices.size(); j += 3)
            {
                Triangle t;
                t.m_indices[0] = indices[j] + min_idx;
                t.m_indices[1] = indices[j+1] + min_idx;
                t.m_indices[2] = indices[j+2] + min_idx;
                p.AddTriangle(t);
            }

            min_idx += pos.size();
        }
    }
    else
    {
        //An error loading the OBJ occurred!
        std::cout << errors << std::endl;
    }
    return p;
}


void MainWindow::on_actionSave_Image_triggered()
{
    QString filename = QFileDialog::getSaveFileName(0, QString("Save Image"), QString("../.."), QString("*.bmp"));
    QString ext = filename.right(4);
    if(QString::compare(ext, QString(".bmp")) != 0)
    {
        filename.append(QString(".bmp"));
    }
    QImageWriter writer(filename);
    writer.setFormat("bmp");
    if(!writer.write(rendered_image))
    {
        qDebug() << writer.errorString();
    }
}

void MainWindow::on_actionEquilateral_Triangle_triggered()
{
    std::vector<glm::vec4> pos;
    pos.push_back(glm::vec4(384,382,0,1));
    pos.push_back(glm::vec4(256,160,0,1));
    pos.push_back(glm::vec4(128,382,0,1));

    std::vector<glm::vec3> col;
    col.push_back(glm::vec3(0,0,255));
    col.push_back(glm::vec3(0,255,0));
    col.push_back(glm::vec3(255,0,0));

    Polygon p(QString("Equilateral"), pos, col);

    p.ClearTriangles();

    Triangle t;
    for(unsigned int i = 0; i < 3; i++)
    {
        t.m_indices[i] = i;
    }

    p.AddTriangle(t);
    std::vector<Polygon> vec; vec.push_back(p);

    rasterizer = Rasterizer(vec);

    rendered_image = rasterizer.renderScene();
    DisplayQImage(rendered_image);
}

void MainWindow::on_actionQuit_Esc_triggered()
{
    QApplication::exit();
}

void MainWindow::on_actionBlinnPhong_triggered() {
    if (!hasLoadScene) {
        return;
    }
    rasterizer.setShadingMode(ShadingMode::BlinnPhong);
    rendered_image = rasterizer.renderScene();
    DisplayQImage(rendered_image);
}

void MainWindow::on_actionWireframes_triggered() {
    if (!hasLoadScene) {
        return;
    }
    rasterizer.setShadingMode(ShadingMode::Wireframes);
    rendered_image = rasterizer.renderScene();
    DisplayQImage(rendered_image);
}
