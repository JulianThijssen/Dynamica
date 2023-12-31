#pragma once

#include "OpenGL.h"
#include "Texture.h"

#include <vector>

#ifdef GDT_NAMESPACE
namespace GDT
{
#endif
    class DrawBuffer;

    class Framebuffer
    {
    public:
        enum BindTarget
        {
            READ = GL_READ_FRAMEBUFFER,
            WRITE = GL_DRAW_FRAMEBUFFER,
            BOTH = GL_FRAMEBUFFER
        };

        Framebuffer() :
            _handle(0),
            colorTexture(MAX_COLOR_ATTACHMENTS)
        {

        }

        void create();
        void destroy();

        void bind() const;
        void bind(BindTarget bindTarget) const;
        void release() const;

        void blitToFramebuffer(Framebuffer target, int srcX0, int srcY0, int srcX1, int srcY1, int dstX0, int dstY0, int dstX1, int dstY1);

        void clearColorBuffer(unsigned int drawBuffer, float r, float g, float b, float a);
        void clearColorBuffer(unsigned int drawBuffer, int r, int g, int b, int a);
        void clearColorBuffer(unsigned int drawBuffer, uint r, uint g, uint b, uint a);
        void clearDepthBuffer(float depthValue);
        void clearStencilBuffer(int stencilValue);

        void addColorTexture(unsigned int colorAttachment, Texture2D texture);
        void addDepthTexture(Texture2D texture);
        void addDepthStencilTexture(Texture2D texture);
        void setDrawBufferCount(unsigned int drawBufferCount);

        void validate() const;

    private:
        const unsigned int MAX_COLOR_ATTACHMENTS = 8;

        GLuint _handle;

        std::vector<Texture2D> colorTexture;

        Texture2D _depthTexture;
    };
#ifdef GDT_NAMESPACE
}
#endif
