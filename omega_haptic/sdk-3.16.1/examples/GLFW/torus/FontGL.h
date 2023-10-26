/*******************************************************************************
 *
 *  Copyright (C) 2001-2023 Force Dimension, Switzerland.
 *  All Rights Reserved.
 *
 *  Force Dimension SDK 3.16.1
 *
 ******************************************************************************/

#ifndef EXAMPLES_GLFW_TORQUES_FONTGL_H
#define EXAMPLES_GLFW_TORQUES_FONTGL_H

/*******************************************************************************
 *
 * Acknowledgement:
 * the following font data structure, font typefaces and font rendering
 * implementation is adapted from the freeglut project implementation of
 * glutRenderBitmap().
 * Credits to http://freeglut.sourceforge.net
 *
 ******************************************************************************/

/* Font data structure */
struct font_struct
{
    char* name;
    int quantity;
    int height;
    const GLubyte** characters;
    float xorig;
    float yorig;
};

/* Font headers and definitions */
#include "FontHelvetica12.h"
#define HELVETICA12 0

/*******************************************************************************
 *
 * This function renders a character with the given font.
 *
 ******************************************************************************/

void render_character(int a_character,
                      int a_font_id)
{
    static const font_struct *font_list [] = { &FontHelvetica12 };

    const font_struct* font = font_list[a_font_id];
    const GLubyte* face = font->characters[a_character];

    glPushClientAttrib(GL_CLIENT_PIXEL_STORE_BIT);
    glPixelStorei(GL_UNPACK_SWAP_BYTES, GL_FALSE);
    glPixelStorei(GL_UNPACK_LSB_FIRST, GL_FALSE);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glBitmap(face[0], font->height, font->xorig, font->yorig, (float)(face[0]), 0.0, (face + 1));
    glPopClientAttrib();
}

#endif  /* EXAMPLES_GLFW_TORQUES_FONTGL_H */
