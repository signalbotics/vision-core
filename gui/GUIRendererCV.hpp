#ifndef GUIRENDERERCV_HPP
#define GUIRENDERERCV_HPP

#include <thread>
#include <mutex>
#include <condition_variable>

#include "GUIRenderer.hpp"

namespace vision_core{

class GUIRendererCV : public GUIRenderer {

public:
    GUIRendererCV();
    void start(Configs config);
    void display();
    bool isStopRequested() const; 
    void renderLoop();
};

#endif // GUIRENDERERCV_HPP

} // namespace vision_core