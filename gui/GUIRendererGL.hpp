#ifndef GUIRENDERERGL_HPP
#define GUIRENDERERGL_HPP
#ifdef WITH_OPENGL
#include <thread>
#include <mutex>
#include <condition_variable>

#include "GUIRenderer.hpp"

#include <GL/glew.h>  // Ensure you include GLEW or an equivalent loader before GLFW
#include <GLFW/glfw3.h>

namespace vision_core{

class GUIRendererGL : public GUIRenderer {

private:

    GLFWwindow* window = nullptr;
    GLuint textureIDOriginal;   // OpenGL texture ID for original frame
    GLuint textureIDProcessed;
    GLuint VAO, VBO, EBO;       // Vertex Array Object, Vertex Buffer Object, Element Buffer Object
    GLuint shaderProgram;       // Shader program ID
    void createTextures();
    void updateTexture(GLuint textureId, const cv::Mat& image);
    GLuint compileShader(GLenum type, const GLchar* source);
    GLuint createShaderProgram(const char* vShaderCode, const char* fShaderCode);
    void setupVertexDataAndBuffers();
    void initOpenGL();
public:
    GUIRendererGL();
    void start(Configs config);
    void display();
    bool isStopRequested() const;
    void renderLoop();
    void closeOpenGL();

};


} // namespace vision_core

#endif // WITH_OPENGL
#endif // GUIRENDERERGL_HPP