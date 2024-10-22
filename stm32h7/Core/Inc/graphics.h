#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <stdint.h>
#include <stdbool.h>

// Framebuffer dimensions
#define FRAMEBUFFER_WIDTH  480
#define FRAMEBUFFER_HEIGHT 480

// Color type definition (RGB565)
typedef uint16_t color_t;

// Framebuffer array (externally defined in the implementation file)
extern color_t framebuffer[FRAMEBUFFER_HEIGHT][FRAMEBUFFER_WIDTH];

// Base Drawable structure
typedef struct Drawable {
    void (*draw)(struct Drawable *self);
    struct Drawable *next;
    bool dirty; // Flag indicating if the object needs to be redrawn
} Drawable;

// Line structure
typedef struct {
    Drawable base; // Inherit from Drawable
    int x0, y0, x1, y1;
    color_t color;
} Line;

// Rectangle structure
typedef struct {
    Drawable base; // Inherit from Drawable
    int x, y, width, height;
    color_t color;
} Rectangle;

// Text structure
typedef struct {
    Drawable base; // Inherit from Drawable
    int x, y;
    const char *text;
    color_t color;
} Text;

// Layer structure
typedef struct {
    Drawable *head; // Pointer to the first drawable in the list
    Drawable *tail; // Pointer to the last drawable in the list (new addition)
} Layer;

// Function prototypes


// Constructors
Line* createLine(int x0, int y0, int x1, int y1, uint16_t color);
Rectangle* createRectangle(int x, int y, int width, int height, uint16_t color);
Text* createText(int x, int y, const char *text, uint16_t color);


// Utility Functions
color_t rgbToColor(uint8_t r, uint8_t g, uint8_t b);
void setPixel(int x, int y, color_t color);

// Drawing Functions for Drawables
void drawLineObject(Drawable *self);
void drawRectangleObject(Drawable *self);
void drawTextObject(Drawable *self);
void drawChar(int x, int y, char c, color_t color);

// Layer Management
void addDrawable(Layer *layer, Drawable *obj);
void drawLayer(Layer *layer);
void drawLayerAlways(Layer *layer);
void clearLayer(Layer *layer);

#endif // GRAPHICS_H
