#include "graphics.h"
#include <stdlib.h>
#include <string.h>

// Framebuffer array
color_t framebuffer[FRAMEBUFFER_HEIGHT][FRAMEBUFFER_WIDTH];

Text* createText(int x, int y, const char *text, uint16_t color) {
    Text *txt = (Text *)malloc(sizeof(Text));
    if (txt) {
        txt->base.draw = drawTextObject;
        txt->base.dirty = true; // Mark as new, so it will be drawn
        txt->base.next = NULL;
        txt->x = x;
        txt->y = y;
        txt->text = strdup(text); // Make a copy of the string
        txt->color = color;
    }
    return txt;
}

Rectangle* createRectangle(int x, int y, int width, int height, uint16_t color) {
    Rectangle *rect = (Rectangle *)malloc(sizeof(Rectangle));
    if (rect) {
        rect->base.draw = drawRectangleObject;
        rect->base.dirty = true; // Mark as new, so it will be drawn
        rect->base.next = NULL;
        rect->x = x;
        rect->y = y;
        rect->width = width;
        rect->height = height;
        rect->color = color;
    }
    return rect;
}

Line* createLine(int x0, int y0, int x1, int y1, uint16_t color) {
    Line *line = (Line *)malloc(sizeof(Line));
    if (line) {
        line->base.draw = drawLineObject;
        line->base.dirty = true; // Mark as new, so it will be drawn
        line->base.next = NULL;
        line->x0 = x0;
        line->y0 = y0;
        line->x1 = x1;
        line->y1 = y1;
        line->color = color;
    }
    return line;
}


// Helper to convert RGB to RGB565 color
color_t rgbToColor(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0x1F) << 11) | ((g & 0x3F) << 5) | (b & 0x1F);
}

// Function to set a pixel in the framebuffer
void setPixel(int x, int y, color_t color) {
    if (x >= 0 && x < FRAMEBUFFER_WIDTH && y >= 0 && y < FRAMEBUFFER_HEIGHT) {
        framebuffer[y][x] = color;
    }
}

// Drawing functions for each object type
void drawLineObject(Drawable *self) {
    Line *line = (Line *)self;
    int dx = abs(line->x1 - line->x0);
    int dy = abs(line->y1 - line->y0);
    int sx = (line->x0 < line->x1) ? 1 : -1;
    int sy = (line->y0 < line->y1) ? 1 : -1;
    int err = dx - dy;

    int x0 = line->x0, y0 = line->y0;
    while (1) {
        setPixel(x0, y0, line->color);
        if (x0 == line->x1 && y0 == line->y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

void drawRectangleObject(Drawable *self) {
    Rectangle *rect = (Rectangle *)self;
    for (int i = 0; i < rect->height; i++) {
        for (int j = 0; j < rect->width; j++) {
            setPixel(rect->x + j, rect->y + i, rect->color);
        }
    }
}

void drawTextObject(Drawable *self) {
    Text *text = (Text *)self;
    int cursorX = text->x;
    int cursorY = text->y;
    while (*text->text) {
        drawChar(cursorX, cursorY, *text->text, text->color);
        cursorX += 8; // Move cursor to the right, assuming 8x8 font
        text->text++;
    }
}

// Drawing single character helper (similar to previous example)
void drawChar(int x, int y, char c, color_t color) {
    // Dummy implementation - You need to link this with actual font drawing code
    if (c < 0x20 || c > 0x7F) return;
    // Assume font bitmap loaded somewhere
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            setPixel(x + j, y + i, color);
        }
    }
}

// Layer Management
void addDrawable(Layer *layer, Drawable *obj) {
    obj->next = NULL;
    if (layer->head == NULL) {
        layer->head = obj;
        layer->tail = obj;
    } else {
        layer->tail->next = obj;
        layer->tail = obj;
    }
}

void drawLayer(Layer *layer) {
    Drawable *current = layer->head;
    while (current != NULL) {
        if (current->dirty) {
            current->draw(current);
            current->dirty = false; // Reset the flag after drawing
        }
        current = current->next;
    }
}

void drawLayerAlways(Layer *layer) {
	Drawable *current = layer->head;
	while (current != NULL) {
		current->draw(current);
		current = current->next;
	}
}

void clearLayer(Layer *layer) {
    Drawable *current = layer->head;
    while (current != NULL) {
        Drawable *next = current->next;
        free(current);
        current = next;
    }
    layer->head = NULL;
}
