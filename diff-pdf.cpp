/*
 * This file is part of diff-pdf.
 *
 * Copyright (C) 2009 TT-Solutions.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "bmpviewer.h"
#include "gutter.h"

#include <stdio.h>
#include <assert.h>

#include <vector>
#include <algorithm>
#include <cmath>
#include <climits>

#include <glib.h>
#include <poppler.h>
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>

#include <wx/app.h>
#include <wx/evtloop.h>
#include <wx/cmdline.h>
#include <wx/filename.h>
#include <wx/log.h>
#include <wx/frame.h>
#include <wx/sizer.h>
#include <wx/toolbar.h>
#include <wx/artprov.h>
#include <wx/progdlg.h>
#include <wx/filesys.h>

// ------------------------------------------------------------------------
// PDF rendering functions
// ------------------------------------------------------------------------

bool g_verbose = false;
bool g_skip_identical = false;
bool g_mark_differences = false;
long g_channel_tolerance = 0;
long g_per_page_pixel_tolerance = 0;
long g_shift_tolerance = 0;  // Object-based shift tolerance: ignore shifted objects within this pixel distance
bool g_grayscale = false;
bool g_draw_object_bbox = false;  // Draw bounding boxes around detected objects
bool g_show_background_in_separate = false;  // Show original PDF in background for additions/removals output
// Resolution to use for rasterization, in DPI
#define DEFAULT_RESOLUTION 300
long g_resolution = DEFAULT_RESOLUTION;

// ------------------------------------------------------------------------
// Connected Component Analysis structures and functions for object-based shift tolerance
// ------------------------------------------------------------------------

struct Component
{
    std::vector<std::pair<int, int> > pixels;  // List of (x,y) coordinates
    int centroid_x, centroid_y;  // Center of mass
    int min_x, min_y, max_x, max_y;  // Bounding box
    int area;  // Pixel count
    
    Component() : centroid_x(0), centroid_y(0), min_x(INT_MAX), min_y(INT_MAX), 
                  max_x(INT_MIN), max_y(INT_MIN), area(0) {}
    
    void add_pixel(int x, int y)
    {
        pixels.push_back(std::make_pair(x, y));
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
        area++;
    }
    
    void calculate_centroid()
    {
        if (area == 0)
        {
            centroid_x = 0;
            centroid_y = 0;
            return;
        }
        
        long long sum_x = 0, sum_y = 0;
        for (size_t i = 0; i < pixels.size(); i++)
        {
            sum_x += pixels[i].first;
            sum_y += pixels[i].second;
        }
        centroid_x = (int)(sum_x / area);
        centroid_y = (int)(sum_y / area);
    }
};

// Forward declaration
std::vector<Component> extract_components(const std::vector<std::vector<bool> >& diff_map, int width, int height);

// Extract all objects from an image surface (non-white pixels grouped into components)
// This identifies all geometric/textual entities in the image
std::vector<Component> extract_all_objects(cairo_surface_t *surface, const wxRect& rect, int offset_x, int offset_y)
{
    std::vector<Component> components;
    if (!surface)
        return components;
    
    int width = cairo_image_surface_get_width(surface);
    int height = cairo_image_surface_get_height(surface);
    int stride = cairo_image_surface_get_stride(surface);
    const unsigned char *data = cairo_image_surface_get_data(surface);
    
    // Build a map of non-white pixels in the diff coordinate space
    int map_width = rect.width;
    int map_height = rect.height;
    std::vector<std::vector<bool> > object_map(map_height, std::vector<bool>(map_width, false));
    
    // Threshold for considering a pixel as part of an object (not white background)
    const int white_threshold = 250;  // RGB values above this are considered white
    
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            const unsigned char *pixel = data + y * stride + x * 4;
            unsigned char r = pixel[0];
            unsigned char g = pixel[1];
            unsigned char b = pixel[2];
            
            // Consider pixel as part of object if it's not white
            bool is_object = (r < white_threshold || g < white_threshold || b < white_threshold);
            
            if (is_object)
            {
                // Convert to diff coordinate space (rdiff is at 0,0 after offset)
                int diff_x = offset_x + x;
                int diff_y = offset_y + y;
                
                // rdiff is at (0,0) after offset, so map directly
                if (diff_x >= 0 && diff_x < rect.width &&
                    diff_y >= 0 && diff_y < rect.height)
                {
                    object_map[diff_y][diff_x] = true;
                }
            }
        }
    }
    
    // Extract components from the object map
    return extract_components(object_map, map_width, map_height);
}

// Extract connected components from a binary difference map using 8-connected flood fill
std::vector<Component> extract_components(const std::vector<std::vector<bool> >& diff_map, int width, int height)
{
    std::vector<Component> components;
    std::vector<std::vector<bool> > visited(height, std::vector<bool>(width, false));
    
    // 8-connected neighbors: (dx, dy) pairs
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (diff_map[y][x] && !visited[y][x])
            {
                // Found a new component, do flood fill
                Component comp;
                std::vector<std::pair<int, int> > stack;
                stack.push_back(std::make_pair(x, y));
                visited[y][x] = true;
                
                while (!stack.empty())
                {
                    std::pair<int, int> p = stack.back();
                    stack.pop_back();
                    int px = p.first;
                    int py = p.second;
                    
                    comp.add_pixel(px, py);
                    
                    // Check all 8 neighbors
                    for (int i = 0; i < 8; i++)
                    {
                        int nx = px + dx[i];
                        int ny = py + dy[i];
                        
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height &&
                            diff_map[ny][nx] && !visited[ny][nx])
                        {
                            visited[ny][nx] = true;
                            stack.push_back(std::make_pair(nx, ny));
                        }
                    }
                }
                
                comp.calculate_centroid();
                components.push_back(comp);
            }
        }
    }
    
    return components;
}

// Match components between two images using first-principles approach:
// 1. Components represent geometric/textual entities (grouped contiguous pixels)
// 2. If a component in image1 appears at a shifted position in image2, match them
// 3. Matching criteria: similar size + centroid within shift tolerance + bounding box overlap
std::vector<std::pair<int, int> > match_components(
    const std::vector<Component>& comp1, 
    const std::vector<Component>& comp2,
    int shift_tolerance,
    const char* debug_label = NULL)
{
    std::vector<std::pair<int, int> > matches;
    std::vector<bool> used2(comp2.size(), false);
    
    if (shift_tolerance <= 0)
        return matches;
    
    if (g_verbose && debug_label)
    {
        printf("DEBUG [%s]: Matching %zu components from set1 with %zu components from set2 (tolerance=%d)\n",
               debug_label, comp1.size(), comp2.size(), shift_tolerance);
    }
    
    for (size_t i = 0; i < comp1.size(); i++)
    {
        int best_match = -1;
        double best_distance = shift_tolerance + 1;
        
        if (g_verbose && debug_label)
        {
            printf("DEBUG [%s]: Component1[%zu]: area=%d, centroid=(%d,%d), bbox=[%d,%d]-[%d,%d]\n",
                   debug_label, i, comp1[i].area, comp1[i].centroid_x, comp1[i].centroid_y,
                   comp1[i].min_x, comp1[i].min_y, comp1[i].max_x, comp1[i].max_y);
        }
        
        for (size_t j = 0; j < comp2.size(); j++)
        {
            if (used2[j])
                continue;
            
            // Calculate Euclidean distance between centroids
            int dx = comp1[i].centroid_x - comp2[j].centroid_x;
            int dy = comp1[i].centroid_y - comp2[j].centroid_y;
            double distance = sqrt(dx * dx + dy * dy);
            
            // CRITICAL: Components at the same position (distance=0) should NOT match
            // Shift tolerance only applies to entities that have actually shifted
            if (distance < 0.5)  // Same position (within rounding error)
                continue;
            
            // Require similar size - components should be within reasonable range
            // Relaxed: allow up to 3x size difference (for cases where objects expand/shrink slightly)
            int area1 = comp1[i].area;
            int area2 = comp2[j].area;
            int max_area = std::max(area1, area2);
            int min_area = std::min(area1, area2);
            bool similar_size = (max_area == 0) || (min_area * 3 >= max_area);  // Relaxed from 2x to 3x
            
            // Relaxed bounding box check: allow partial overlap (at least 50% of smaller bbox)
            int shift_x = comp2[j].centroid_x - comp1[i].centroid_x;
            int shift_y = comp2[j].centroid_y - comp1[i].centroid_y;
            
            // Calculate overlap area of shifted bboxes
            int overlap_x1 = std::max(comp1[i].min_x + shift_x, comp2[j].min_x);
            int overlap_y1 = std::max(comp1[i].min_y + shift_y, comp2[j].min_y);
            int overlap_x2 = std::min(comp1[i].max_x + shift_x, comp2[j].max_x);
            int overlap_y2 = std::min(comp1[i].max_y + shift_y, comp2[j].max_y);
            int overlap_area = 0;
            if (overlap_x1 < overlap_x2 && overlap_y1 < overlap_y2)
            {
                overlap_area = (overlap_x2 - overlap_x1 + 1) * (overlap_y2 - overlap_y1 + 1);
            }
            
            int comp1_bbox_area = (comp1[i].max_x - comp1[i].min_x + 1) * (comp1[i].max_y - comp1[i].min_y + 1);
            int comp2_bbox_area = (comp2[j].max_x - comp2[j].min_x + 1) * (comp2[j].max_y - comp2[j].min_y + 1);
            int min_bbox_area = std::min(comp1_bbox_area, comp2_bbox_area);
            
            // Require at least 30% overlap of the smaller bbox (relaxed from full overlap)
            bool bboxes_overlap = (min_bbox_area > 0) && (overlap_area * 10 >= min_bbox_area * 3);
            
            bool matches_criteria = (distance <= shift_tolerance && distance < best_distance && similar_size && bboxes_overlap);
            
            if (g_verbose && debug_label && distance <= shift_tolerance * 2)  // Show candidates within 2x tolerance
            {
                printf("  vs Component2[%zu]: area=%d, centroid=(%d,%d), distance=%.2f, similar_size=%d, bbox_overlap=%d -> %s\n",
                       j, area2, comp2[j].centroid_x, comp2[j].centroid_y, distance,
                       similar_size ? 1 : 0, bboxes_overlap ? 1 : 0,
                       matches_criteria ? "MATCH" : "reject");
            }
            
            if (matches_criteria)
            {
                best_match = j;
                best_distance = distance;
            }
        }
        
        if (best_match >= 0)
        {
            matches.push_back(std::make_pair(i, best_match));
            used2[best_match] = true;
            if (g_verbose && debug_label)
            {
                printf("DEBUG [%s]: -> MATCHED Component1[%zu] with Component2[%zu] (distance=%.2f)\n",
                       debug_label, i, (size_t)best_match, best_distance);
            }
        }
        else if (g_verbose && debug_label)
        {
            printf("DEBUG [%s]: -> NO MATCH for Component1[%zu]\n", debug_label, i);
        }
    }
    
    if (g_verbose && debug_label)
    {
        printf("DEBUG [%s]: Total matches: %zu\n", debug_label, matches.size());
    }
    
    return matches;
}

// ------------------------------------------------------------------------

inline unsigned char to_grayscale(unsigned char r, unsigned char g, unsigned char b)
{
    return (unsigned char)(0.2126 * r + 0.7152 * g + 0.0722 * b);
}

inline bool pixel_has_content(unsigned char r, unsigned char g, unsigned char b)
{
    // Treat very light pixels as background (same threshold as connected component extraction)
    return to_grayscale(r, g, b) < 250;
}

cairo_surface_t *render_page(PopplerPage *page)
{
    double w, h;
    poppler_page_get_size(page, &w, &h);

    const int w_px = int((int)g_resolution * w / 72.0);
    const int h_px = int((int)g_resolution * h / 72.0);

    cairo_surface_t *surface =
        cairo_image_surface_create(CAIRO_FORMAT_RGB24, w_px, h_px);

    cairo_t *cr = cairo_create(surface);

    // clear the surface to white background:
    cairo_save(cr);
    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_rectangle(cr, 0, 0, w_px, h_px);
    cairo_fill(cr);
    cairo_restore(cr);

    // Scale so that PDF output covers the whole surface. Image surface is
    // created with transformation set up so that 1 coordinate unit is 1 pixel;
    // Poppler assumes 1 unit = 1 point.
    cairo_scale(cr, (int)g_resolution / 72.0, (int)g_resolution / 72.0);

    poppler_page_render(page, cr);

    cairo_show_page(cr);

    cairo_destroy(cr);

    return surface;
}


// Creates image showing only additions (content in s2 that differs from s1).
// Returns NULL if there are no additions.
cairo_surface_t *diff_additions(int page, cairo_surface_t *s1, cairo_surface_t *s2,
                                int offset_x = 0, int offset_y = 0)
{
    if ( !s2 )
        return NULL;

    long pixel_diff_count = 0;
    wxRect r1, r2;

    if ( s1 )
    {
        r1 = wxRect(0, 0,
                    cairo_image_surface_get_width(s1),
                    cairo_image_surface_get_height(s1));
    }
    r2 = wxRect(offset_x, offset_y,
                cairo_image_surface_get_width(s2),
                cairo_image_surface_get_height(s2));

    // compute union rectangle starting at [0,0] position
    wxRect rdiff(r1);
    rdiff.Union(r2);
    r1.Offset(-rdiff.x, -rdiff.y);
    r2.Offset(-rdiff.x, -rdiff.y);
    rdiff.Offset(-rdiff.x, -rdiff.y);

    bool has_additions = false;
    cairo_surface_t *result =
        cairo_image_surface_create(CAIRO_FORMAT_RGB24, rdiff.width, rdiff.height);

    const int stride1 = s1 ? cairo_image_surface_get_stride(s1) : 0;
    const int stride2 = cairo_image_surface_get_stride(s2);
    const int strideresult = cairo_image_surface_get_stride(result);

    const unsigned char *data1 = s1 ? cairo_image_surface_get_data(s1) : NULL;
    const unsigned char *data2 = cairo_image_surface_get_data(s2);
    unsigned char *dataresult = cairo_image_surface_get_data(result);

    if ( g_show_background_in_separate )
    {
        // Copy s2 (new PDF) as background
        unsigned char *out = dataresult + r2.y * strideresult + r2.x * 4;
        const unsigned char *in = data2;
        for ( int y = 0; y < r2.height; y++, in += stride2, out += strideresult )
        {
            memcpy(out, in, r2.width * 4);
        }
    }
    else
    {
        // Start with white background
        cairo_t *cr = cairo_create(result);
        cairo_set_source_rgb(cr, 1, 1, 1);
        cairo_rectangle(cr, 0, 0, rdiff.width, rdiff.height);
        cairo_fill(cr);
        cairo_destroy(cr);
    }

    // First pass: build binary difference map
    std::vector<std::vector<bool> > diff_map(rdiff.height, std::vector<bool>(rdiff.width, false));
    
    const unsigned char *data2_scan = data2;
    for ( int y = 0; y < r2.height; y++ )
    {
        for ( int x = 0; x < r2.width; x++ )
        {
            int px = r2.x + x;
            int py = r2.y + y;
            if ( px < 0 || px >= rdiff.width || py < 0 || py >= rdiff.height )
                continue;
                
            unsigned char cr2 = *(data2_scan + x * 4 + 0);
            unsigned char cg2 = *(data2_scan + x * 4 + 1);
            unsigned char cb2 = *(data2_scan + x * 4 + 2);
            bool s2_has_content = pixel_has_content(cr2, cg2, cb2);

            bool is_addition = false;
            
            // Check if this pixel is in s1's area
            if ( s1 && px >= r1.x && px < r1.x + r1.width && py >= r1.y && py < r1.y + r1.height )
            {
                // Compare with s1
                int s1_x = px - r1.x;
                int s1_y = py - r1.y;
                const unsigned char *s1_data = data1 + s1_y * stride1 + s1_x * 4;
                unsigned char cr1 = *(s1_data + 0);
                unsigned char cg1 = *(s1_data + 1);
                unsigned char cb1 = *(s1_data + 2);

                bool pixels_differ = ( cr1 > (cr2+g_channel_tolerance) || cr1 < (cr2-g_channel_tolerance)
                                     || cg1 > (cg2+g_channel_tolerance) || cg1 < (cg2-g_channel_tolerance)
                                     || cb1 > (cb2+g_channel_tolerance) || cb1 < (cb2-g_channel_tolerance) );

                if ( pixels_differ && s2_has_content )
                {
                    is_addition = true;
                }
            }
            else
            {
                // s1 doesn't cover this area, so it's an addition if s2 has content
                if ( s2_has_content )
                    is_addition = true;
            }

            if ( is_addition )
            {
                diff_map[py][px] = true;
            }
        }
        data2_scan += stride2;
    }

    // Extract components and match if shift tolerance is enabled
    std::vector<bool> pixel_matched(rdiff.width * rdiff.height, false);
    std::vector<Component> addition_components;  // Store for bbox drawing
    std::vector<bool> addition_matched;  // Track which components are matched
    
    if ( g_shift_tolerance > 0 && s1 && s2 )
    {
        // NEW APPROACH: Extract ALL objects from both images first, then match by centroid
        // This correctly identifies shifted labels/objects that should be ignored
        
        // Extract all objects from s1 (1B)
        std::vector<Component> s1_all_objects = extract_all_objects(s1, rdiff, r1.x, r1.y);
        if ( g_verbose )
            printf("DEBUG [additions]: Extracted %zu objects from s1 (1B)\n", s1_all_objects.size());
        
        // Extract all objects from s2 (1C)
        std::vector<Component> s2_all_objects = extract_all_objects(s2, rdiff, r2.x, r2.y);
        if ( g_verbose )
            printf("DEBUG [additions]: Extracted %zu objects from s2 (1C)\n", s2_all_objects.size());
        
        // Match objects between s1 and s2 by centroid
        std::vector<std::pair<int, int> > object_matches = match_components(s1_all_objects, s2_all_objects, g_shift_tolerance, "object-matching");
        if ( g_verbose )
            printf("DEBUG [additions]: Matched %zu object pairs between s1 and s2\n", object_matches.size());
        
        // Build a set of matched object pixels (in diff coordinate space)
        for ( size_t m = 0; m < object_matches.size(); m++ )
        {
            // Mark pixels from matched s2 objects (these are shifted but same object)
            const Component& s2_obj = s2_all_objects[object_matches[m].second];
            for ( size_t p = 0; p < s2_obj.pixels.size(); p++ )
            {
                int px = s2_obj.pixels[p].first;
                int py = s2_obj.pixels[p].second;
                if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height )
                    pixel_matched[py * rdiff.width + px] = true;
            }
        }
        
        // Extract components from additions diff_map (for display/bbox)
        addition_components = extract_components(diff_map, rdiff.width, rdiff.height);
        if ( g_verbose )
            printf("DEBUG [additions]: Extracted %zu addition components from diff_map\n", addition_components.size());
        
        // Track which addition components overlap with matched objects
        addition_matched.assign(addition_components.size(), false);
        for ( size_t i = 0; i < addition_components.size(); i++ )
        {
            const Component& comp = addition_components[i];
            bool overlaps_matched = false;
            for ( size_t p = 0; p < comp.pixels.size(); p++ )
            {
                int px = comp.pixels[p].first;
                int py = comp.pixels[p].second;
                if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height &&
                     pixel_matched[py * rdiff.width + px] )
                {
                    overlaps_matched = true;
                    break;
                }
            }
            addition_matched[i] = overlaps_matched;
        }
    }
    else if ( g_draw_object_bbox )
    {
        // Extract components even if shift tolerance is disabled (for bbox drawing)
        addition_components = extract_components(diff_map, rdiff.width, rdiff.height);
        addition_matched.assign(addition_components.size(), false);
    }

    // Second pass: copy pixels only if they're in unmatched components
    // Also filter out components that are too small (noise)
    const int min_component_area_for_output = 10;  // Don't render components smaller than this
    
    // Create a lookup map: pixel coordinate -> component index
    // This avoids O(n*m*k) nested loops when rendering
    std::vector<int> pixel_to_component(rdiff.width * rdiff.height, -1);
    std::vector<int> component_visible_pixel_count(addition_components.size(), 0);
    std::vector<bool> component_should_render(addition_components.size(), false);
    
    // Build lookup map and count visible pixels per component
    for ( size_t i = 0; i < addition_components.size(); i++ )
    {
        const Component& comp = addition_components[i];
        if ( comp.area < min_component_area_for_output )
            continue;  // Skip very small components
        
        for ( size_t p = 0; p < comp.pixels.size(); p++ )
        {
            int px = comp.pixels[p].first;
            int py = comp.pixels[p].second;
            if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height )
            {
                pixel_to_component[py * rdiff.width + px] = i;
                
                // Count visible unmatched pixels
                if ( diff_map[py][px] && !pixel_matched[py * rdiff.width + px] )
                {
                    component_visible_pixel_count[i]++;
                }
            }
        }
        
        // Mark component for rendering if it has enough visible pixels
        if ( component_visible_pixel_count[i] >= 3 )
        {
            component_should_render[i] = true;
        }
    }
    
    unsigned char *out = dataresult + r2.y * strideresult + r2.x * 4;
    data2_scan = data2;
    for ( int y = 0; y < r2.height; y++, data2_scan += stride2, out += strideresult )
    {
        for ( int x = 0; x < r2.width * 4; x += 4 )
        {
            int px = r2.x + x/4;
            int py = r2.y + y;
            
            if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height &&
                 diff_map[py][px] && !pixel_matched[py * rdiff.width + px] )
            {
                // Fast O(1) lookup to find component
                int component_idx = pixel_to_component[py * rdiff.width + px];
                if ( component_idx >= 0 && component_should_render[component_idx] )
                {
                    has_additions = true;
                    pixel_diff_count++;
                    // Use uniform dark green for additions, same as in -diff
                    // Note: Cairo RGB24 format uses BGR byte order (B=+0, G=+1, R=+2)
                    const unsigned char dark_green = 120;  // Uniform dark green intensity
                    *(out + x + 0) = 0;              // Blue = 0 (BGR format)
                    *(out + x + 1) = dark_green;     // Green = uniform dark green (BGR format)
                    *(out + x + 2) = 0;              // Red = 0 (BGR format)
                }
                // else leave white (already filled) - component too small
            }
            // else leave white (already filled)
        }
    }

    // Draw bounding boxes around unmatched components if requested
    if ( g_draw_object_bbox && pixel_diff_count > 0 && !addition_components.empty() )
    {
        cairo_t *cr_bbox = cairo_create(result);
        // Use light gray color for bounding boxes
        cairo_set_source_rgba(cr_bbox, 0.75, 0.75, 0.75, 0.3);
        cairo_set_line_width(cr_bbox, 2.0);
        
        const int min_component_area = 10;  // Ignore components smaller than this (noise)
        const int min_visible_pixel_diff = 20;  // Pixels must differ by at least this much to be visible
        
        // Draw boxes only for unmatched components that have actual visible pixels in output
        for ( size_t i = 0; i < addition_components.size(); i++ )
        {
            // Skip if matched or if component is too small
            if ( !addition_matched.empty() && addition_matched[i] )
                continue;
            
            const Component& comp = addition_components[i];
            
            // Skip very small components (noise)
            if ( comp.area < min_component_area )
                continue;
            
            // Check if this component has any visible pixels that are in diff_map AND not matched
            // (these are the pixels that will actually be rendered)
            bool has_visible_unmatched_pixels = false;
            int visible_pixel_count = 0;
            for ( size_t p = 0; p < comp.pixels.size(); p++ )
            {
                int px = comp.pixels[p].first;
                int py = comp.pixels[p].second;
                if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height )
                {
                    // Check if pixel is in diff_map and not matched
                    if ( diff_map[py][px] && !pixel_matched[py * rdiff.width + px] )
                    {
                        // Check if pixel is actually visible (not too close to white/background)
                        if ( px >= r2.x && px < r2.x + r2.width &&
                             py >= r2.y && py < r2.y + r2.height )
                        {
                            int s2_x = px - r2.x;
                            int s2_y = py - r2.y;
                            const unsigned char *s2_data = data2 + s2_y * stride2 + s2_x * 4;
                            unsigned char r = s2_data[0];
                            unsigned char g = s2_data[1];
                            unsigned char b = s2_data[2];
                            
                            // Pixel is visible if it's significantly different from white
                            // (at least one channel is more than min_visible_pixel_diff away from 255)
                            if ( (255 - r) >= min_visible_pixel_diff || 
                                 (255 - g) >= min_visible_pixel_diff || 
                                 (255 - b) >= min_visible_pixel_diff )
                            {
                                visible_pixel_count++;
                                has_visible_unmatched_pixels = true;
                            }
                        }
                    }
                }
            }
            
            // Only draw bbox if component has visible unmatched pixels and enough of them
            if ( has_visible_unmatched_pixels && visible_pixel_count >= 3 )
            {
                cairo_rectangle(cr_bbox, comp.min_x, comp.min_y, 
                               comp.max_x - comp.min_x + 1, 
                               comp.max_y - comp.min_y + 1);
                cairo_stroke(cr_bbox);
            }
        }
        
        cairo_destroy(cr_bbox);
    }

    if ( g_verbose )
        printf("page %d has %ld addition pixels\n", page, pixel_diff_count);

    // Only return result if we actually found addition pixels
    // If everything was matched (pixel_diff_count == 0), return NULL
    if ( pixel_diff_count > 0 && (g_per_page_pixel_tolerance == 0 || pixel_diff_count > g_per_page_pixel_tolerance) )
    {
        return result;
    }
    else
    {
        cairo_surface_destroy(result);
        return NULL;
    }
}

// Creates image showing only removals (content in s1 that differs from s2).
// Returns NULL if there are no removals.
cairo_surface_t *diff_removals(int page, cairo_surface_t *s1, cairo_surface_t *s2,
                               int offset_x = 0, int offset_y = 0)
{
    if ( !s1 )
        return NULL;

    long pixel_diff_count = 0;
    wxRect r1, r2;

    r1 = wxRect(0, 0,
                cairo_image_surface_get_width(s1),
                cairo_image_surface_get_height(s1));
    if ( s2 )
    {
        r2 = wxRect(offset_x, offset_y,
                    cairo_image_surface_get_width(s2),
                    cairo_image_surface_get_height(s2));
    }

    // compute union rectangle starting at [0,0] position
    wxRect rdiff(r1);
    if ( s2 )
        rdiff.Union(r2);
    r1.Offset(-rdiff.x, -rdiff.y);
    if ( s2 )
        r2.Offset(-rdiff.x, -rdiff.y);
    rdiff.Offset(-rdiff.x, -rdiff.y);

    bool has_removals = false;
    cairo_surface_t *result =
        cairo_image_surface_create(CAIRO_FORMAT_RGB24, rdiff.width, rdiff.height);

    const int stride1 = cairo_image_surface_get_stride(s1);
    const int stride2 = s2 ? cairo_image_surface_get_stride(s2) : 0;
    const int strideresult = cairo_image_surface_get_stride(result);

    const unsigned char *data1 = cairo_image_surface_get_data(s1);
    const unsigned char *data2 = s2 ? cairo_image_surface_get_data(s2) : NULL;
    unsigned char *dataresult = cairo_image_surface_get_data(result);

    if ( g_show_background_in_separate )
    {
        // Copy s1 (old PDF) as background
        unsigned char *out = dataresult + r1.y * strideresult + r1.x * 4;
        const unsigned char *in = data1;
        for ( int y = 0; y < r1.height; y++, in += stride1, out += strideresult )
        {
            memcpy(out, in, r1.width * 4);
        }
    }
    else
    {
        // Start with white background
        cairo_t *cr = cairo_create(result);
        cairo_set_source_rgb(cr, 1, 1, 1);
        cairo_rectangle(cr, 0, 0, rdiff.width, rdiff.height);
        cairo_fill(cr);
        cairo_destroy(cr);
    }

    // First pass: build binary difference map
    std::vector<std::vector<bool> > diff_map(rdiff.height, std::vector<bool>(rdiff.width, false));
    
    const unsigned char *data1_scan = data1;
    for ( int y = 0; y < r1.height; y++ )
    {
        for ( int x = 0; x < r1.width; x++ )
        {
            int px = r1.x + x;
            int py = r1.y + y;
            if ( px < 0 || px >= rdiff.width || py < 0 || py >= rdiff.height )
                continue;
                
            unsigned char cr1 = *(data1_scan + x * 4 + 0);
            unsigned char cg1 = *(data1_scan + x * 4 + 1);
            unsigned char cb1 = *(data1_scan + x * 4 + 2);
            bool s1_has_content = pixel_has_content(cr1, cg1, cb1);

            bool is_removal = false;
            
            // Check if this pixel is in s2's area
            if ( s2 && px >= r2.x && px < r2.x + r2.width && py >= r2.y && py < r2.y + r2.height )
            {
                // Compare with s2
                int s2_x = px - r2.x;
                int s2_y = py - r2.y;
                const unsigned char *s2_data = data2 + s2_y * stride2 + s2_x * 4;
                unsigned char cr2 = *(s2_data + 0);
                unsigned char cg2 = *(s2_data + 1);
                unsigned char cb2 = *(s2_data + 2);

                bool pixels_differ = ( cr1 > (cr2+g_channel_tolerance) || cr1 < (cr2-g_channel_tolerance)
                                     || cg1 > (cg2+g_channel_tolerance) || cg1 < (cg2-g_channel_tolerance)
                                     || cb1 > (cb2+g_channel_tolerance) || cb1 < (cb2-g_channel_tolerance) );

                if ( pixels_differ && s1_has_content )
                {
                    is_removal = true;
                }
            }
            else
            {
                // s2 doesn't cover this area, so it's a removal if s1 has content
                if ( s1_has_content )
                    is_removal = true;
            }

            if ( is_removal )
            {
                diff_map[py][px] = true;
            }
        }
        data1_scan += stride1;
    }

    // Extract components and match if shift tolerance is enabled
    std::vector<bool> pixel_matched(rdiff.width * rdiff.height, false);
    std::vector<Component> removal_components;  // Store for bbox drawing
    std::vector<bool> removal_matched;  // Track which components are matched
    
    if ( g_shift_tolerance > 0 && s1 && s2 )
    {
        // NEW APPROACH: Extract ALL objects from both images first, then match by centroid
        // This correctly identifies shifted labels/objects that should be ignored
        
        // Extract all objects from s1 (1B)
        std::vector<Component> s1_all_objects = extract_all_objects(s1, rdiff, r1.x, r1.y);
        if ( g_verbose )
            printf("DEBUG [removals]: Extracted %zu objects from s1 (1B)\n", s1_all_objects.size());
        
        // Extract all objects from s2 (1C)
        std::vector<Component> s2_all_objects = extract_all_objects(s2, rdiff, r2.x, r2.y);
        if ( g_verbose )
            printf("DEBUG [removals]: Extracted %zu objects from s2 (1C)\n", s2_all_objects.size());
        
        // Match objects between s1 and s2 by centroid
        std::vector<std::pair<int, int> > object_matches = match_components(s1_all_objects, s2_all_objects, g_shift_tolerance, "object-matching");
        if ( g_verbose )
            printf("DEBUG [removals]: Matched %zu object pairs between s1 and s2\n", object_matches.size());
        
        // Build a set of matched object pixels (in diff coordinate space)
        for ( size_t m = 0; m < object_matches.size(); m++ )
        {
            // Mark pixels from matched s1 objects (these are shifted but same object)
            const Component& s1_obj = s1_all_objects[object_matches[m].first];
            for ( size_t p = 0; p < s1_obj.pixels.size(); p++ )
            {
                int px = s1_obj.pixels[p].first;
                int py = s1_obj.pixels[p].second;
                if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height )
                    pixel_matched[py * rdiff.width + px] = true;
            }
        }
        
        // Extract components from removals diff_map (for display/bbox)
        removal_components = extract_components(diff_map, rdiff.width, rdiff.height);
        if ( g_verbose )
            printf("DEBUG [removals]: Extracted %zu removal components from diff_map\n", removal_components.size());
        
        // Track which removal components overlap with matched objects
        removal_matched.assign(removal_components.size(), false);
        for ( size_t i = 0; i < removal_components.size(); i++ )
        {
            const Component& comp = removal_components[i];
            bool overlaps_matched = false;
            for ( size_t p = 0; p < comp.pixels.size(); p++ )
            {
                int px = comp.pixels[p].first;
                int py = comp.pixels[p].second;
                if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height &&
                     pixel_matched[py * rdiff.width + px] )
                {
                    overlaps_matched = true;
                    break;
                }
            }
            removal_matched[i] = overlaps_matched;
        }
    }
    else if ( g_draw_object_bbox )
    {
        // Extract components even if shift tolerance is disabled (for bbox drawing)
        removal_components = extract_components(diff_map, rdiff.width, rdiff.height);
        removal_matched.assign(removal_components.size(), false);
    }

    // Second pass: copy pixels only if they're in unmatched components
    // Also filter out components that are too small (noise)
    const int min_component_area_for_output = 10;  // Don't render components smaller than this
    
    // Create a lookup map: pixel coordinate -> component index
    // This avoids O(n*m*k) nested loops when rendering
    std::vector<int> pixel_to_component(rdiff.width * rdiff.height, -1);
    std::vector<int> component_visible_pixel_count(removal_components.size(), 0);
    std::vector<bool> component_should_render(removal_components.size(), false);
    
    // Build lookup map and count visible pixels per component
    for ( size_t i = 0; i < removal_components.size(); i++ )
    {
        const Component& comp = removal_components[i];
        if ( comp.area < min_component_area_for_output )
            continue;  // Skip very small components
        
        for ( size_t p = 0; p < comp.pixels.size(); p++ )
        {
            int px = comp.pixels[p].first;
            int py = comp.pixels[p].second;
            if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height )
            {
                pixel_to_component[py * rdiff.width + px] = i;
                
                // Count visible unmatched pixels
                if ( diff_map[py][px] && !pixel_matched[py * rdiff.width + px] )
                {
                    component_visible_pixel_count[i]++;
                }
            }
        }
        
        // Mark component for rendering if it has enough visible pixels
        if ( component_visible_pixel_count[i] >= 3 )
        {
            component_should_render[i] = true;
        }
    }
    
    unsigned char *out = dataresult + r1.y * strideresult + r1.x * 4;
    data1_scan = data1;
    for ( int y = 0; y < r1.height; y++, data1_scan += stride1, out += strideresult )
    {
        for ( int x = 0; x < r1.width * 4; x += 4 )
        {
            int px = r1.x + x/4;
            int py = r1.y + y;
            
            if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height &&
                 diff_map[py][px] && !pixel_matched[py * rdiff.width + px] )
            {
                // Fast O(1) lookup to find component
                int component_idx = pixel_to_component[py * rdiff.width + px];
                if ( component_idx >= 0 && component_should_render[component_idx] )
                {
                    has_removals = true;
                    pixel_diff_count++;
                    // Use red color for removals, same as in -diff
                    // Convert to grayscale first, then apply red tint
                    // Note: Cairo RGB24 format uses BGR byte order (B=+0, G=+1, R=+2)
                    unsigned char cr1 = *(data1_scan + x + 0);
                    unsigned char cg1 = *(data1_scan + x + 1);
                    unsigned char cb1 = *(data1_scan + x + 2);
                    unsigned char gray = (unsigned char)(0.299 * cr1 + 0.587 * cg1 + 0.114 * cb1);
                    *(out + x + 0) = 0;      // Blue = 0 (BGR format)
                    *(out + x + 1) = 0;      // Green = 0 (BGR format)
                    *(out + x + 2) = gray;   // Red = grayscale value (BGR format)
                }
                // else leave white (already filled) - component too small
            }
            // else leave white (already filled)
        }
    }

    // Draw bounding boxes around unmatched components if requested
    if ( g_draw_object_bbox && pixel_diff_count > 0 && !removal_components.empty() )
    {
        cairo_t *cr_bbox = cairo_create(result);
        // Use light gray color for bounding boxes
        cairo_set_source_rgba(cr_bbox, 0.75, 0.75, 0.75, 0.3);
        cairo_set_line_width(cr_bbox, 2.0);
        
        const int min_component_area = 10;  // Ignore components smaller than this (noise)
        const int min_visible_pixel_diff = 20;  // Pixels must differ by at least this much to be visible
        
        // Draw boxes only for unmatched components that have actual visible pixels in output
        for ( size_t i = 0; i < removal_components.size(); i++ )
        {
            // Skip if matched or if component is too small
            if ( !removal_matched.empty() && removal_matched[i] )
                continue;
            
            const Component& comp = removal_components[i];
            
            // Skip very small components (noise)
            if ( comp.area < min_component_area )
                continue;
            
            // Check if this component has any visible pixels that are in diff_map AND not matched
            // (these are the pixels that will actually be rendered)
            bool has_visible_unmatched_pixels = false;
            int visible_pixel_count = 0;
            for ( size_t p = 0; p < comp.pixels.size(); p++ )
            {
                int px = comp.pixels[p].first;
                int py = comp.pixels[p].second;
                if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height )
                {
                    // Check if pixel is in diff_map and not matched
                    if ( diff_map[py][px] && !pixel_matched[py * rdiff.width + px] )
                    {
                        // Check if pixel is actually visible (not too close to white/background)
                        if ( px >= r1.x && px < r1.x + r1.width &&
                             py >= r1.y && py < r1.y + r1.height )
                        {
                            int s1_x = px - r1.x;
                            int s1_y = py - r1.y;
                            const unsigned char *s1_data = data1 + s1_y * stride1 + s1_x * 4;
                            unsigned char r = s1_data[0];
                            unsigned char g = s1_data[1];
                            unsigned char b = s1_data[2];
                            
                            // Pixel is visible if it's significantly different from white
                            // (at least one channel is more than min_visible_pixel_diff away from 255)
                            if ( (255 - r) >= min_visible_pixel_diff || 
                                 (255 - g) >= min_visible_pixel_diff || 
                                 (255 - b) >= min_visible_pixel_diff )
                            {
                                visible_pixel_count++;
                                has_visible_unmatched_pixels = true;
                            }
                        }
                    }
                }
            }
            
            // Only draw bbox if component has visible unmatched pixels and enough of them
            if ( has_visible_unmatched_pixels && visible_pixel_count >= 3 )
            {
                cairo_rectangle(cr_bbox, comp.min_x, comp.min_y, 
                               comp.max_x - comp.min_x + 1, 
                               comp.max_y - comp.min_y + 1);
                cairo_stroke(cr_bbox);
            }
        }
        
        cairo_destroy(cr_bbox);
    }

    if ( g_verbose )
        printf("page %d has %ld removal pixels\n", page, pixel_diff_count);

    // Only return result if we actually found removal pixels
    // If everything was matched (pixel_diff_count == 0), return NULL
    if ( pixel_diff_count > 0 && (g_per_page_pixel_tolerance == 0 || pixel_diff_count > g_per_page_pixel_tolerance) )
    {
        return result;
    }
    else
    {
        cairo_surface_destroy(result);
        return NULL;
    }
}

// Creates image of differences between s1 and s2. If the offset is specified,
// then s2 is displaced by it. If thumbnail and thumbnail_width are specified,
// then a thumbnail with highlighted differences is created too.
cairo_surface_t *diff_images(int page, cairo_surface_t *s1, cairo_surface_t *s2,
                             int offset_x = 0, int offset_y = 0,
                             wxImage *thumbnail = NULL, int thumbnail_width = -1)
{
    assert( s1 || s2 );

    long pixel_diff_count = 0;
    wxRect r1, r2;

    if ( s1 )
    {
        r1 = wxRect(0, 0,
                    cairo_image_surface_get_width(s1),
                    cairo_image_surface_get_height(s1));
    }
    if ( s2 )
    {
        r2 = wxRect(offset_x, offset_y,
                    cairo_image_surface_get_width(s2),
                    cairo_image_surface_get_height(s2));
    }

    // compute union rectangle starting at [0,0] position
    wxRect rdiff(r1);
    rdiff.Union(r2);
    r1.Offset(-rdiff.x, -rdiff.y);
    r2.Offset(-rdiff.x, -rdiff.y);
    rdiff.Offset(-rdiff.x, -rdiff.y);

    bool changes = false;

    cairo_surface_t *diff =
        cairo_image_surface_create(CAIRO_FORMAT_RGB24, rdiff.width, rdiff.height);

    float thumbnail_scale;
    int thumbnail_height;

    if ( thumbnail )
    {
        thumbnail_scale = float(thumbnail_width) / float(rdiff.width);
        thumbnail_height = int(rdiff.height * thumbnail_scale);
        thumbnail->Create(thumbnail_width, thumbnail_height);
        // initalize the thumbnail with a white rectangle:
        thumbnail->SetRGB(wxRect(), 255, 255, 255);
    }

    // clear the surface to white background if the merged images don't fully
    // overlap:
    if ( r1 != r2 )
    {
        changes = true;

        cairo_t *cr = cairo_create(diff);
        cairo_set_source_rgb(cr, 1, 1, 1);
        cairo_rectangle(cr, 0, 0, rdiff.width, rdiff.height);
        cairo_fill(cr);
        cairo_destroy(cr);
    }

    const int stride1 = s1 ? cairo_image_surface_get_stride(s1) : 0;
    const int stride2 = s2 ? cairo_image_surface_get_stride(s2) : 0;
    const int stridediff = cairo_image_surface_get_stride(diff);

    const unsigned char *data1 = s1 ? cairo_image_surface_get_data(s1) : NULL;
    const unsigned char *data2 = s2 ? cairo_image_surface_get_data(s2) : NULL;
    unsigned char *datadiff = cairo_image_surface_get_data(diff);

    // we visualize the differences by taking one channel from s1
    // and the other two channels from s2:

    // first, copy s1 over:
    if ( s1 )
    {
        unsigned char *out = datadiff + r1.y * stridediff + r1.x * 4;
        for ( int y = 0;
              y < r1.height;
              y++, data1 += stride1, out += stridediff )
        {
            memcpy(out, data1, r1.width * 4);
        }
    }

    // Build difference map and track additions vs removals
    std::vector<std::vector<bool> > diff_map(rdiff.height, std::vector<bool>(rdiff.width, false));
    std::vector<std::vector<bool> > is_addition_map(rdiff.height, std::vector<bool>(rdiff.width, false));
    std::vector<std::vector<bool> > is_removal_map(rdiff.height, std::vector<bool>(rdiff.width, false));
    
    if ( s2 )
    {
        unsigned char *out = datadiff + r2.y * stridediff + r2.x * 4;
        const unsigned char *data2_scan = data2;
        for ( int y = 0; y < r2.height; y++ )
        {
            for ( int x = 0; x < r2.width; x++ )
            {
                int px = r2.x + x;
                int py = r2.y + y;
                if ( px < 0 || px >= rdiff.width || py < 0 || py >= rdiff.height )
                    continue;
                    
                unsigned char cr1 = *(out + x * 4 + 0);
                unsigned char cg1 = *(out + x * 4 + 1);
                unsigned char cb1 = *(out + x * 4 + 2);

                unsigned char cr2 = *(data2_scan + x * 4 + 0);
                unsigned char cg2 = *(data2_scan + x * 4 + 1);
                unsigned char cb2 = *(data2_scan + x * 4 + 2);

                bool pixels_differ = ( cr1 > (cr2+g_channel_tolerance) || cr1 < (cr2-g_channel_tolerance)
                                     || cg1 > (cg2+g_channel_tolerance) || cg1 < (cg2-g_channel_tolerance)
                                     || cb1 > (cb2+g_channel_tolerance) || cb1 < (cb2-g_channel_tolerance) );

                if ( pixels_differ )
                {
                    diff_map[py][px] = true;
                    
                    // Determine if this is an addition or removal
                    // Check if s1 pixel is white (background) vs s2 has content
                    unsigned char gray_s1 = (unsigned char)(0.299 * cr1 + 0.587 * cg1 + 0.114 * cb1);
                    unsigned char gray_s2 = (unsigned char)(0.299 * cr2 + 0.587 * cg2 + 0.114 * cb2);
                    
                    bool s1_is_background = (gray_s1 > 250);  // s1 is white/background
                    bool s2_is_background = (gray_s2 > 250);  // s2 is white/background
                    
                    if (s2_is_background && !s1_is_background)
                    {
                        // Removal: s1 has content, s2 is background
                        is_removal_map[py][px] = true;
                    }
                    else if (!s2_is_background && s1_is_background)
                    {
                        // Addition: s2 has content, s1 is background
                        is_addition_map[py][px] = true;
                    }
                    else if (!s2_is_background && !s1_is_background)
                    {
                        // Both have content but differ - use intensity to determine
                        if (gray_s2 > gray_s1 + 5)
                            is_addition_map[py][px] = true;  // s2 is brighter -> addition
                        else if (gray_s1 > gray_s2 + 5)
                            is_removal_map[py][px] = true;  // s1 is brighter -> removal
                        else
                            is_addition_map[py][px] = true;  // Default to addition if unclear
                    }
                }
            }
            data2_scan += stride2;
            out += stridediff;
        }
    }
    
    // Also check for removals in s1 that don't overlap with s2
    if ( s1 )
    {
        for ( int y = 0; y < r1.height; y++ )
        {
            for ( int x = 0; x < r1.width; x++ )
            {
                int px = r1.x + x;
                int py = r1.y + y;
                if ( px < 0 || px >= rdiff.width || py < 0 || py >= rdiff.height )
                    continue;
                    
                // Check if this area is not covered by s2
                if ( !s2 || px < r2.x || px >= r2.x + r2.width || py < r2.y || py >= r2.y + r2.height )
                {
                    // This is a removal (s1 has content but s2 doesn't cover this area)
                    const unsigned char *s1_data = data1 + y * stride1 + x * 4;
                    unsigned char cr1 = s1_data[0];
                    unsigned char cg1 = s1_data[1];
                    unsigned char cb1 = s1_data[2];
                    unsigned char gray_s1 = (unsigned char)(0.299 * cr1 + 0.587 * cg1 + 0.114 * cb1);
                    
                    if (gray_s1 < 250)  // s1 has content
                    {
                        diff_map[py][px] = true;
                        is_removal_map[py][px] = true;
                    }
                }
            }
        }
    }

    // Extract components and match if shift tolerance is enabled
    // Use the same object-based matching as diff_additions and diff_removals
    std::vector<bool> pixel_matched(rdiff.width * rdiff.height, false);
    
    // Declare component filtering structures outside the if block so they're accessible during rendering
    std::vector<int> pixel_to_component(rdiff.width * rdiff.height, -1);
    std::vector<bool> component_should_render;
    
    if ( g_shift_tolerance > 0 && s1 && s2 )
    {
        // NEW APPROACH: Extract ALL objects from both images first, then match by centroid
        // This correctly identifies shifted labels/objects that should be ignored
        
        // Extract all objects from s1
        std::vector<Component> s1_all_objects = extract_all_objects(s1, rdiff, r1.x, r1.y);
        if ( g_verbose )
            printf("DEBUG [diff_images]: Extracted %zu objects from s1\n", s1_all_objects.size());
        
        // Extract all objects from s2
        std::vector<Component> s2_all_objects = extract_all_objects(s2, rdiff, r2.x, r2.y);
        if ( g_verbose )
            printf("DEBUG [diff_images]: Extracted %zu objects from s2\n", s2_all_objects.size());
        
        // Match objects between s1 and s2 by centroid
        std::vector<std::pair<int, int> > object_matches = match_components(s1_all_objects, s2_all_objects, g_shift_tolerance, "diff-images");
        if ( g_verbose )
        {
            printf("DEBUG [diff_images]: Matched %zu object pairs between s1 and s2\n", object_matches.size());
            for ( size_t m = 0; m < object_matches.size(); m++ )
            {
                const Component& s1_obj = s1_all_objects[object_matches[m].first];
                const Component& s2_obj = s2_all_objects[object_matches[m].second];
                int dx = s2_obj.centroid_x - s1_obj.centroid_x;
                int dy = s2_obj.centroid_y - s1_obj.centroid_y;
                double dist = sqrt(dx*dx + dy*dy);
                printf("DEBUG [diff_images]: Match %zu: s1[%d] area=%d centroid=(%d,%d) <-> s2[%d] area=%d centroid=(%d,%d) dist=%.2f\n",
                       m, object_matches[m].first, s1_obj.area, s1_obj.centroid_x, s1_obj.centroid_y,
                       object_matches[m].second, s2_obj.area, s2_obj.centroid_x, s2_obj.centroid_y, dist);
            }
        }
        
        // Build a set of matched object pixels (in diff coordinate space)
        // Reuse the exact same logic as diff_additions and diff_removals
        // Mark pixels from BOTH matched objects (old and new positions)
        for ( size_t m = 0; m < object_matches.size(); m++ )
        {
            // Mark pixels from matched s1 objects (old position - removals)
            const Component& s1_obj = s1_all_objects[object_matches[m].first];
            for ( size_t p = 0; p < s1_obj.pixels.size(); p++ )
            {
                int px = s1_obj.pixels[p].first;
                int py = s1_obj.pixels[p].second;
                if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height )
                    pixel_matched[py * rdiff.width + px] = true;
            }
            // Mark pixels from matched s2 objects (new position - additions)
            const Component& s2_obj = s2_all_objects[object_matches[m].second];
            for ( size_t p = 0; p < s2_obj.pixels.size(); p++ )
            {
                int px = s2_obj.pixels[p].first;
                int py = s2_obj.pixels[p].second;
                if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height )
                    pixel_matched[py * rdiff.width + px] = true;
            }
        }
        
        // Extract components from diff_map for filtering small components (same as diff_additions/diff_removals)
        std::vector<Component> diff_components = extract_components(diff_map, rdiff.width, rdiff.height);
        if ( g_verbose )
            printf("DEBUG [diff_images]: Extracted %zu difference components from diff_map\n", diff_components.size());
        
        // Filter out small components (noise) - reuse exact logic from diff_additions
        const int min_component_area_for_output = 10;  // Don't render components smaller than this
        
        // Initialize component filtering structures (pixel_to_component already declared above)
        std::vector<int> component_visible_pixel_count(diff_components.size(), 0);
        component_should_render.assign(diff_components.size(), false);
        
        // Build lookup map and count visible pixels per component
        for ( size_t i = 0; i < diff_components.size(); i++ )
        {
            const Component& comp = diff_components[i];
            if ( comp.area < min_component_area_for_output )
                continue;  // Skip very small components
            
            for ( size_t p = 0; p < comp.pixels.size(); p++ )
            {
                int px = comp.pixels[p].first;
                int py = comp.pixels[p].second;
                if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height )
                {
                    pixel_to_component[py * rdiff.width + px] = i;
                    
                    // Count visible unmatched pixels
                    if ( diff_map[py][px] && !pixel_matched[py * rdiff.width + px] )
                    {
                        component_visible_pixel_count[i]++;
                    }
                }
            }
            
            // Mark component for rendering if it has enough visible pixels
            if ( component_visible_pixel_count[i] >= 3 )
            {
                component_should_render[i] = true;
            }
        }
        
        // Update pixel_matched to also exclude small components
        for ( size_t i = 0; i < diff_components.size(); i++ )
        {
            if ( !component_should_render[i] )
            {
                const Component& comp = diff_components[i];
                for ( size_t p = 0; p < comp.pixels.size(); p++ )
                {
                    int px = comp.pixels[p].first;
                    int py = comp.pixels[p].second;
                    if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height )
                        pixel_matched[py * rdiff.width + px] = true;  // Mark as matched (don't render)
                }
            }
        }
    }

    // Now render with matched components ignored
    if ( s2 )
    {
        unsigned char *out = datadiff + r2.y * stridediff + r2.x * 4;
        for ( int y = 0;
              y < r2.height;
              y++, data2 += stride2, out += stridediff )
        {
            bool linediff = false;

            for ( int x = 0; x < r2.width * 4; x += 4 )
            {
                int px = r2.x + x/4;
                int py = r2.y + y;
                
                // Reuse exact same logic as diff_additions: check diff_map AND not matched
                // AND component should be rendered (if shift tolerance enabled)
                bool pixels_differ = false;
                if ( px >= 0 && px < rdiff.width && py >= 0 && py < rdiff.height &&
                     diff_map[py][px] && !pixel_matched[py * rdiff.width + px] )
                {
                    // If shift tolerance is enabled, also check component filtering
                    if ( g_shift_tolerance > 0 && s1 && s2 )
                    {
                        // Fast O(1) lookup to find component
                        int component_idx = pixel_to_component[py * rdiff.width + px];
                        if ( component_idx >= 0 && component_should_render[component_idx] )
                        {
                            pixels_differ = true;
                        }
                        // else: component too small or filtered out
                    }
                    else
                    {
                        pixels_differ = true;
                    }
                }

                if ( pixels_differ )
                {
                    pixel_diff_count++;
                    changes = true;
                    linediff = true;

                    if ( thumbnail )
                    {
                        // calculate the coordinates in the thumbnail
                        int tx = int((r2.x + x/4.0) * thumbnail_scale);
                        int ty = int((r2.y + y) * thumbnail_scale);

                        // Limit the coordinates to the thumbnail size (may be
                        // off slightly due to rounding errors).
                        // See https://github.com/vslavik/diff-pdf/pull/58
                        tx = std::min(tx, thumbnail_width - 1);
                        ty = std::min(ty, thumbnail_height - 1);

                        // mark changes with red
                        thumbnail->SetRGB(tx, ty, 255, 0, 0);
                    }
                    
                    // Only modify pixel data for visualization if it's a real difference
                    // (not a matched/shifted object)
                    unsigned char cr1 = *(out + x + 0);
                    unsigned char cg1 = *(out + x + 1);
                    unsigned char cb1 = *(out + x + 2);
                    unsigned char cr2 = *(data2 + x + 0);
                    unsigned char cg2 = *(data2 + x + 1);
                    unsigned char cb2 = *(data2 + x + 2);

                    if (g_grayscale)
                    {
                        // convert both images to grayscale, use blue for s1, red for s2
                        unsigned char gray1 = to_grayscale(cr1, cg1, cb1);
                        unsigned char gray2 = to_grayscale(cr2, cg2, cb2);
                        *(out + x + 0) = gray2;
                        *(out + x + 1) = (gray1 + gray2) / 2;
                        *(out + x + 2) = gray1;
                    }
                    else
                    {
                        // Visualize differences: Dark green for additions, Red for removals
                        // Use the pre-computed addition/removal maps
                        unsigned char gray_s1 = (unsigned char)(0.299 * cr1 + 0.587 * cg1 + 0.114 * cb1);
                        unsigned char gray_s2 = (unsigned char)(0.299 * cr2 + 0.587 * cg2 + 0.114 * cb2);
                        
                        // Note: Cairo RGB24 format uses BGR byte order (B=+0, G=+1, R=+2)
                        const unsigned char dark_green = 120;  // Uniform dark green intensity
                        if (is_addition_map[py][px])
                        {
                            // Addition: Uniform dark green (BGR format: B=0, G=value, R=0)
                            *(out + x + 0) = 0;              // Blue = 0
                            *(out + x + 1) = dark_green;     // Green = uniform dark green
                            *(out + x + 2) = 0;              // Red = 0
                        }
                        else if (is_removal_map[py][px])
                        {
                            // Removal: Red (BGR format: B=0, G=0, R=value)
                            *(out + x + 0) = 0;              // Blue = 0
                            *(out + x + 1) = 0;              // Green = 0
                            *(out + x + 2) = gray_s1;        // Red = s1 intensity
                        }
                        else
                        {
                            // Fallback: shouldn't happen, but default to addition (green)
                            *(out + x + 0) = 0;
                            *(out + x + 1) = dark_green;
                            *(out + x + 2) = 0;
                        }
                    }
                }
                // else: pixel is matched (shifted object) or not a difference - leave it unchanged from s1
            }

            if (g_mark_differences && linediff)
            {
                for (int x = 0; x < (10 < r2.width ? 10 : r2.width) * 4; x+=4)
                {
                   *(out + x + 0) = 0;
                   *(out + x + 1) = 0;
                   *(out + x + 2) = 255;
                }
            }
        }
    }

    // add background image of the page to the thumbnails
    if ( thumbnail )
    {
        // copy the 'diff' surface into wxImage:
        wxImage bg(rdiff.width, rdiff.height);
        unsigned char *in = datadiff;
        unsigned char *out = bg.GetData();
        for ( int y = 0; y < rdiff.height; y++, in += stridediff )
        {
            for ( int x = 0; x < rdiff.width * 4; x += 4 )
            {
                // cairo_surface_t uses BGR order, wxImage has RGB
                *(out++) = *(in + x + 2);
                *(out++) = *(in + x + 1);
                *(out++) = *(in + x + 0);
            }
        }

        // scale it to thumbnail size:
        bg.Rescale(thumbnail_width, thumbnail_height, wxIMAGE_QUALITY_HIGH);

        // and merge with the diff markers in *thumbnail, making it much
        // lighter in the process:
        in = bg.GetData();
        out = thumbnail->GetData();
        for ( int i = thumbnail_width * thumbnail_height; i > 0; i-- )
        {
            if ( out[1] == 0 ) // G=0 ==> not white
            {
                // marked with red color, as place with differences -- don't
                // paint background image here, make the red as visible as
                // possible
                out += 3;
                in += 3;
            }
            else
            {
                // merge in lighter background image
                *(out++) = 128 + *(in++) / 2;
                *(out++) = 128 + *(in++) / 2;
                *(out++) = 128 + *(in++) / 2;
            }
        }

        // If there were no changes, indicate it by using green
        // (170,230,130) color for the thumbnail in gutter control:
        if ( !changes )
        {
            out = thumbnail->GetData();
            for ( int i = thumbnail_width * thumbnail_height;
                  i > 0;
                  i--, out += 3 )
            {
                out[0] = 170/2 + out[0] / 2;
                out[1] = 230/2 + out[1] / 2;
                out[2] = 130/2 + out[2] / 2;
            }
        }
    }

    if ( g_verbose )
        printf("page %d has %ld pixels that differ\n", page, pixel_diff_count);

    // If we specified a tolerance, then return if we have exceeded that for this page
    if ( g_per_page_pixel_tolerance == 0 ? changes : pixel_diff_count > g_per_page_pixel_tolerance)
    {
        return diff;
    }
    else
    {
        cairo_surface_destroy(diff);
        return NULL;
    }
}


// Compares given two pages for separate additions/removals output.
// If cr_additions is not NULL, additions are drawn to it.
// If cr_removals is not NULL, removals are drawn to it.
bool page_compare_separate(int page,
                           cairo_t *cr_additions, cairo_t *cr_removals,
                           PopplerPage *page1, PopplerPage *page2)
{
    cairo_surface_t *img1 = page1 ? render_page(page1) : NULL;
    cairo_surface_t *img2 = page2 ? render_page(page2) : NULL;

    cairo_surface_t *additions = diff_additions(page, img1, img2, 0, 0);
    cairo_surface_t *removals = diff_removals(page, img1, img2, 0, 0);
    
    const bool has_additions = (additions != NULL);
    const bool has_removals = (removals != NULL);
    const bool has_diff = has_additions || has_removals;

    if ( cr_additions )
    {
        if ( additions )
        {
            cairo_save(cr_additions);
            cairo_scale(cr_additions, 72.0 / g_resolution, 72.0 / g_resolution);
            cairo_set_source_surface(cr_additions, additions, 0, 0);
            cairo_paint(cr_additions);
            cairo_restore(cr_additions);
        }
        else if ( !g_skip_identical && page2 )
        {
            // If no additions but we want all pages, render original page2
            poppler_page_render(page2, cr_additions);
        }
        
        if ( additions || (!g_skip_identical && page2) )
            cairo_show_page(cr_additions);
    }

    if ( cr_removals )
    {
        if ( removals )
        {
            cairo_save(cr_removals);
            cairo_scale(cr_removals, 72.0 / g_resolution, 72.0 / g_resolution);
            cairo_set_source_surface(cr_removals, removals, 0, 0);
            cairo_paint(cr_removals);
            cairo_restore(cr_removals);
        }
        else if ( !g_skip_identical && page1 )
        {
            // If no removals but we want all pages, render original page1
            poppler_page_render(page1, cr_removals);
        }
        
        if ( removals || (!g_skip_identical && page1) )
            cairo_show_page(cr_removals);
    }

    if ( additions )
        cairo_surface_destroy(additions);
    if ( removals )
        cairo_surface_destroy(removals);

    if ( img1 )
        cairo_surface_destroy(img1);
    if ( img2 )
        cairo_surface_destroy(img2);

    return !has_diff;
}

// Compares given two pages. If cr_out is not NULL, then the diff image (either
// differences or unmodified page, if there are no diffs) is drawn to it.
// If thumbnail and thumbnail_width are specified, then a thumbnail with
// highlighted differences is created too.
bool page_compare(int page, cairo_t *cr_out,
                  PopplerPage *page1, PopplerPage *page2,
                  wxImage *thumbnail = NULL, int thumbnail_width = -1)
{
    cairo_surface_t *img1 = page1 ? render_page(page1) : NULL;
    cairo_surface_t *img2 = page2 ? render_page(page2) : NULL;

    cairo_surface_t *diff = diff_images(page, img1, img2, 0, 0,
                                        thumbnail, thumbnail_width);
    const bool has_diff = (diff != NULL);

    if ( cr_out )
    {
        if ( diff )
        {
            // render the difference as high-resolution bitmap

            cairo_save(cr_out);
            cairo_scale(cr_out, 72.0 / g_resolution, 72.0 / g_resolution);

            cairo_set_source_surface(cr_out, diff ? diff : img1, 0, 0);
            cairo_paint(cr_out);

            cairo_restore(cr_out);
        }
        else
        {
            // save space (as well as improve rendering quality) in diff pdf
            // by writing unchanged pages in their original form rather than
            // a rasterized one

            if (!g_skip_identical)
               poppler_page_render(page1, cr_out);
        }

        if (diff || !g_skip_identical)
            cairo_show_page(cr_out);
    }

    if ( diff )
        cairo_surface_destroy(diff);

    if ( img1 )
        cairo_surface_destroy(img1);
    if ( img2 )
        cairo_surface_destroy(img2);

    return !has_diff;
}


// Compares two documents, writing additions and removals to separate PDF files.
// 'pdf_additions' and 'pdf_removals' can be NULL if that output is not needed.
bool doc_compare_separate(PopplerDocument *doc1, PopplerDocument *doc2,
                          const char *pdf_additions, const char *pdf_removals,
                          std::vector<bool> *differences = NULL,
                          wxProgressDialog *progress = NULL)
{
    int pages_differ = 0;

    cairo_surface_t *surface_additions = NULL;
    cairo_t *cr_additions = NULL;
    cairo_surface_t *surface_removals = NULL;
    cairo_t *cr_removals = NULL;

    if ( pdf_additions )
    {
        double w, h;
        poppler_page_get_size(poppler_document_get_page(doc2 ? doc2 : doc1, 0), &w, &h);
        surface_additions = cairo_pdf_surface_create(pdf_additions, w, h);
        cr_additions = cairo_create(surface_additions);
    }

    if ( pdf_removals )
    {
        double w, h;
        poppler_page_get_size(poppler_document_get_page(doc1, 0), &w, &h);
        surface_removals = cairo_pdf_surface_create(pdf_removals, w, h);
        cr_removals = cairo_create(surface_removals);
    }

    int pages1 = poppler_document_get_n_pages(doc1);
    int pages2 = poppler_document_get_n_pages(doc2);
    int pages_total = pages1 > pages2 ? pages1 : pages2;

    if ( pages1 != pages2 )
    {
        if ( g_verbose )
            printf("pages count differs: %d vs %d\n", pages1, pages2);
    }

    for ( int page = 0; page < pages_total; page++ )
    {
        if ( progress )
        {
            progress->Update
                      (
                          page,
                          wxString::Format
                          (
                              "Comparing page %d of %d...",
                              page+1,
                              pages_total
                          )
                       );
        }

        PopplerPage *page1 = page < pages1
                             ? poppler_document_get_page(doc1, page)
                             : NULL;
        PopplerPage *page2 = page < pages2
                             ? poppler_document_get_page(doc2, page)
                             : NULL;

        if ( cr_additions && page != 0 )
        {
            double w, h;
            PopplerPage *page_ref = page2 ? page2 : page1;
            if ( page_ref )
            {
                poppler_page_get_size(page_ref, &w, &h);
                cairo_pdf_surface_set_size(surface_additions, w, h);
            }
        }

        if ( cr_removals && page != 0 )
        {
            double w, h;
            if ( page1 )
            {
                poppler_page_get_size(page1, &w, &h);
                cairo_pdf_surface_set_size(surface_removals, w, h);
            }
        }

        bool page_same = page_compare_separate(page, cr_additions, cr_removals,
                                               page1, page2);

        if ( differences )
            differences->push_back(!page_same);

        if ( !page_same )
        {
            pages_differ++;

            if ( g_verbose )
                printf("page %d differs\n", page);
        }
    }

    if ( cr_additions )
    {
        cairo_destroy(cr_additions);
        cairo_surface_destroy(surface_additions);
    }

    if ( cr_removals )
    {
        cairo_destroy(cr_removals);
        cairo_surface_destroy(surface_removals);
    }

    if (g_verbose)
        printf("%d of %d pages differ.\n", pages_differ, pages_total);

    // are doc1 and doc2 the same?
    return (pages_differ == 0) && (pages1 == pages2);
}

// Compares two documents, writing diff PDF into file named 'pdf_output' if
// not NULL. if 'differences' is not NULL, puts a map of which pages differ
// into it. If 'progress' is provided, it is updated to reflect comparison's
// progress. If 'gutter' is set, then all the pages are added to it, with
// their respective thumbnails (the gutter must be empty beforehand).
bool doc_compare(PopplerDocument *doc1, PopplerDocument *doc2,
                 const char *pdf_output,
                 std::vector<bool> *differences,
                 wxProgressDialog *progress = NULL,
                 Gutter *gutter = NULL)
{
    int pages_differ = 0;

    cairo_surface_t *surface_out = NULL;
    cairo_t *cr_out = NULL;

    if ( pdf_output )
    {
        double w, h;
        poppler_page_get_size(poppler_document_get_page(doc1, 0), &w, &h);
        surface_out = cairo_pdf_surface_create(pdf_output, w, h);
        cr_out = cairo_create(surface_out);
    }

    int pages1 = poppler_document_get_n_pages(doc1);
    int pages2 = poppler_document_get_n_pages(doc2);
    int pages_total = pages1 > pages2 ? pages1 : pages2;

    if ( pages1 != pages2 )
    {
        if ( g_verbose )
            printf("pages count differs: %d vs %d\n", pages1, pages2);
    }

    for ( int page = 0; page < pages_total; page++ )
    {
        if ( progress )
        {
            progress->Update
                      (
                          page,
                          wxString::Format
                          (
                              "Comparing page %d of %d...",
                              page+1,
                              pages_total
                          )
                       );
        }

        if ( pdf_output && page != 0 )
        {
            double w, h;
            poppler_page_get_size(poppler_document_get_page(doc1, page), &w, &h);
            cairo_pdf_surface_set_size(surface_out, w, h);
        }

        PopplerPage *page1 = page < pages1
                             ? poppler_document_get_page(doc1, page)
                             : NULL;
        PopplerPage *page2 = page < pages2
                             ? poppler_document_get_page(doc2, page)
                             : NULL;

        bool page_same;

        if ( gutter )
        {
            wxImage thumbnail;
            page_same = page_compare(page, cr_out, page1, page2,
                                     &thumbnail, Gutter::WIDTH);

            wxString label1("(null)");
            wxString label2("(null)");

            if ( page1 )
            {
                gchar *label;
                g_object_get(page1, "label", &label, NULL);
                label1 = wxString::FromUTF8(label);
                g_free(label);
            }
            if ( page2 )
            {
                gchar *label;
                g_object_get(page2, "label", &label, NULL);
                label2 = wxString::FromUTF8(label);
                g_free(label);
            }


            wxString label;
            if ( label1 == label2 )
                label = label1;
            else
                label = label1 + " / " + label2;

            gutter->AddPage(label, thumbnail);
        }
        else
        {
            page_same = page_compare(page, cr_out, page1, page2);
        }

        if ( differences )
            differences->push_back(!page_same);

        if ( !page_same )
        {
	    pages_differ ++;

            if ( g_verbose )
                printf("page %d differs\n", page);

            // If we don't need to output all different pages in any
            // form (including verbose report of differing pages!), then
            // we can stop comparing the PDFs as soon as we find the first
            // difference.
            if ( !g_verbose && !pdf_output && !differences && !gutter )
                break;
        }
    }

    if ( pdf_output )
    {
        cairo_destroy(cr_out);
        cairo_surface_destroy(surface_out);
    }

    if (g_verbose)
        printf("%d of %d pages differ.\n", pages_differ, pages_total);

    // are doc1 and doc1 the same?
    return (pages_differ == 0) && (pages1 == pages2);
}


// ------------------------------------------------------------------------
// wxWidgets GUI
// ------------------------------------------------------------------------

const int ID_PREV_PAGE = wxNewId();
const int ID_NEXT_PAGE = wxNewId();
const int ID_ZOOM_IN = wxNewId();
const int ID_ZOOM_OUT = wxNewId();
const int ID_OFFSET_LEFT = wxNewId();
const int ID_OFFSET_RIGHT = wxNewId();
const int ID_OFFSET_UP = wxNewId();
const int ID_OFFSET_DOWN = wxNewId();
const int ID_GUTTER = wxNewId();

#define BMP_ARTPROV(id) wxArtProvider::GetBitmap(id, wxART_TOOLBAR)

#define BMP_PREV_PAGE      BMP_ARTPROV(wxART_GO_BACK)
#define BMP_NEXT_PAGE      BMP_ARTPROV(wxART_GO_FORWARD)

#define BMP_OFFSET_LEFT    BMP_ARTPROV(wxART_GO_BACK)
#define BMP_OFFSET_RIGHT   BMP_ARTPROV(wxART_GO_FORWARD)
#define BMP_OFFSET_UP      BMP_ARTPROV(wxART_GO_UP)
#define BMP_OFFSET_DOWN    BMP_ARTPROV(wxART_GO_DOWN)

#ifdef __WXGTK__
    #define BMP_ZOOM_IN    BMP_ARTPROV("gtk-zoom-in")
    #define BMP_ZOOM_OUT   BMP_ARTPROV("gtk-zoom-out")
#else
    #include "gtk-zoom-in.xpm"
    #include "gtk-zoom-out.xpm"
    #define BMP_ZOOM_IN    wxBitmap(gtk_zoom_in_xpm)
    #define BMP_ZOOM_OUT   wxBitmap(gtk_zoom_out_xpm)
#endif

static const float ZOOM_FACTOR_STEP = 1.2f;

class DiffFrame : public wxFrame
{
public:
    DiffFrame(const wxString& title)
        : wxFrame(NULL, wxID_ANY, title)
    {
        m_cur_page = -1;

        CreateStatusBar(2);
        SetStatusBarPane(0);
        const int stat_widths[] = { -1, 150 };
        SetStatusWidths(2, stat_widths);

        wxToolBar *toolbar =
            new wxToolBar
                (
                    this, wxID_ANY,
                    wxDefaultPosition, wxDefaultSize,
                    wxTB_HORIZONTAL | wxTB_FLAT | wxTB_HORZ_TEXT
                );

        toolbar->AddTool(ID_PREV_PAGE, "Previous", BMP_PREV_PAGE,
                         "Go to previous page (PgUp)");
        toolbar->AddTool(ID_NEXT_PAGE, "Next", BMP_NEXT_PAGE,
                         "Go to next page (PgDown)");
        toolbar->AddTool(ID_ZOOM_IN, "Zoom in", BMP_ZOOM_IN,
                         "Make the page larger (Ctrl +)");
        toolbar->AddTool(ID_ZOOM_OUT, "Zoom out", BMP_ZOOM_OUT,
                         "Make the page smaller (Ctrl -)");
        toolbar->AddTool(ID_OFFSET_LEFT, "", BMP_OFFSET_LEFT,
                         "Offset one of the pages to the left (Ctrl left)");
        toolbar->AddTool(ID_OFFSET_RIGHT, "", BMP_OFFSET_RIGHT,
                         "Offset one of the pages to the right (Ctrl right)");
        toolbar->AddTool(ID_OFFSET_UP, "", BMP_OFFSET_UP,
                         "Offset one of the pages up (Ctrl up)");
        toolbar->AddTool(ID_OFFSET_DOWN, "", BMP_OFFSET_DOWN,
                         "Offset one of the pages down (Ctrl down)");

        toolbar->Realize();
        SetToolBar(toolbar);

        wxAcceleratorEntry accels[8];
        accels[0].Set(wxACCEL_NORMAL, WXK_PAGEUP, ID_PREV_PAGE);
        accels[1].Set(wxACCEL_NORMAL, WXK_PAGEDOWN, ID_NEXT_PAGE);
        accels[2].Set(wxACCEL_CTRL, (int)'=', ID_ZOOM_IN);
        accels[3].Set(wxACCEL_CTRL, (int)'-', ID_ZOOM_OUT);
        accels[4].Set(wxACCEL_CTRL, WXK_LEFT, ID_OFFSET_LEFT);
        accels[5].Set(wxACCEL_CTRL, WXK_RIGHT, ID_OFFSET_RIGHT);
        accels[6].Set(wxACCEL_CTRL, WXK_UP, ID_OFFSET_UP);
        accels[7].Set(wxACCEL_CTRL, WXK_DOWN, ID_OFFSET_DOWN);

        wxAcceleratorTable accel_table(8, accels);
        SetAcceleratorTable(accel_table);

        m_gutter = new Gutter(this, ID_GUTTER);

        m_viewer = new BitmapViewer(this);
        m_viewer->AttachGutter(m_gutter);
        m_viewer->SetFocus();

        wxBoxSizer *sizer = new wxBoxSizer(wxHORIZONTAL);
        sizer->Add(m_gutter, wxSizerFlags(0).Expand().Border(wxALL, 2));
        sizer->Add(m_viewer, wxSizerFlags(1).Expand());
        SetSizer(sizer);
    }

    void SetDocs(PopplerDocument *doc1, PopplerDocument *doc2)
    {
        m_doc1 = doc1;
        m_doc2 = doc2;

        wxProgressDialog progress("Comparing documents",
                                  "Comparing documents...",
                                  wxMax(poppler_document_get_n_pages(m_doc1),
                                        poppler_document_get_n_pages(m_doc2)),
                                  this,
                                  wxPD_SMOOTH | wxPD_REMAINING_TIME);


        doc_compare(m_doc1, m_doc2, NULL, &m_pages, &progress, m_gutter);

        progress.Pulse();

        m_diff_count = 0;
        for ( std::vector<bool>::const_iterator i = m_pages.begin();
              i != m_pages.end();
              ++i )
        {
            if ( *i )
                m_diff_count++;
        }

        GoToPage(0);

        progress.Pulse();

        m_viewer->SetBestFitZoom();
        UpdateStatus();

        progress.Hide();
    }

    void GoToPage(int n)
    {
        m_cur_page = n;
        m_gutter->SetSelection(n);
        DoUpdatePage();
    }

private:
    void DoUpdatePage()
    {
        wxBusyCursor wait;

        const int pages1 = poppler_document_get_n_pages(m_doc1);
        const int pages2 = poppler_document_get_n_pages(m_doc2);

        PopplerPage *page1 = m_cur_page < pages1
                             ? poppler_document_get_page(m_doc1, m_cur_page)
                             : NULL;
        PopplerPage *page2 = m_cur_page < pages2
                             ? poppler_document_get_page(m_doc2, m_cur_page)
                             : NULL;

        cairo_surface_t *img1 = page1 ? render_page(page1) : NULL;
        cairo_surface_t *img2 = page2 ? render_page(page2) : NULL;

        wxImage thumbnail;
        cairo_surface_t *diff = diff_images
                                (
                                    m_cur_page,
                                    img1, img2,
                                    m_offset.x, m_offset.y,
                                    &thumbnail, Gutter::WIDTH
                                );

        m_viewer->Set(diff ? diff : img1);

        // Always update the diff map. It will be all-white if there were
        // no differences.
        m_gutter->SetThumbnail(m_cur_page, thumbnail);

        if ( img1 )
            cairo_surface_destroy(img1);
        if ( img2 )
            cairo_surface_destroy(img2);
        if ( diff )
            cairo_surface_destroy(diff);

        UpdateStatus();
    }

    void UpdateStatus()
    {
        SetStatusText
        (
            wxString::Format
            (
                "Page %d of %d; %d of them %s different, this page %s",
                m_cur_page + 1 /* humans prefer 1-based counting*/,
                (int)m_pages.size(),
                m_diff_count,
                m_diff_count == 1 ? "is" : "are",
                m_pages[m_cur_page] ? "differs" : "is unchanged"
            ),
            0
        );

        SetStatusText
        (
            wxString::Format
            (
                "%.1f%% [offset %d,%d]",
                m_viewer->GetZoom() * 100.0,
                m_offset.x, m_offset.y
            ),
            1
        );
    }

    void OnSetPage(wxCommandEvent& event)
    {
        GoToPage(event.GetSelection());
    }

    void OnPrevPage(wxCommandEvent&)
    {
        if ( m_cur_page > 0 )
            GoToPage(m_cur_page - 1);
    }

    void OnNextPage(wxCommandEvent&)
    {
        if ( m_cur_page < m_pages.size() - 1 )
            GoToPage(m_cur_page + 1);
    }

    void OnUpdatePrevPage(wxUpdateUIEvent& event)
    {
        event.Enable(m_cur_page > 0);
    }

    void OnUpdateNextPage(wxUpdateUIEvent& event)
    {
        event.Enable(m_cur_page < m_pages.size() - 1);
    }

    void OnZoomIn(wxCommandEvent&)
    {
        wxBusyCursor wait;
        m_viewer->SetZoom(m_viewer->GetZoom() * ZOOM_FACTOR_STEP);
        UpdateStatus();
    }

    void OnZoomOut(wxCommandEvent&)
    {
        wxBusyCursor wait;
        m_viewer->SetZoom(m_viewer->GetZoom() / ZOOM_FACTOR_STEP);
        UpdateStatus();
    }

    void DoOffset(int x, int y)
    {
        m_offset.x += x;
        m_offset.y += y;
        DoUpdatePage();
    }

    void OnOffsetLeft(wxCommandEvent&) { DoOffset(-1, 0); }
    void OnOffsetRight(wxCommandEvent&) { DoOffset(1, 0); }
    void OnOffsetUp(wxCommandEvent&) { DoOffset(0, -1); }
    void OnOffsetDown(wxCommandEvent&) { DoOffset(0, 1); }

    DECLARE_EVENT_TABLE()

private:
    BitmapViewer *m_viewer;
    Gutter *m_gutter;
    PopplerDocument *m_doc1, *m_doc2;
    std::vector<bool> m_pages;
    int m_diff_count;
    int m_cur_page;
    wxPoint m_offset;
};

BEGIN_EVENT_TABLE(DiffFrame, wxFrame)
    EVT_LISTBOX  (ID_GUTTER,       DiffFrame::OnSetPage)
    EVT_TOOL     (ID_PREV_PAGE,    DiffFrame::OnPrevPage)
    EVT_TOOL     (ID_NEXT_PAGE,    DiffFrame::OnNextPage)
    EVT_UPDATE_UI(ID_PREV_PAGE,    DiffFrame::OnUpdatePrevPage)
    EVT_UPDATE_UI(ID_NEXT_PAGE,    DiffFrame::OnUpdateNextPage)
    EVT_TOOL     (ID_ZOOM_IN,      DiffFrame::OnZoomIn)
    EVT_TOOL     (ID_ZOOM_OUT,     DiffFrame::OnZoomOut)
    EVT_TOOL     (ID_OFFSET_LEFT,  DiffFrame::OnOffsetLeft)
    EVT_TOOL     (ID_OFFSET_RIGHT, DiffFrame::OnOffsetRight)
    EVT_TOOL     (ID_OFFSET_UP,    DiffFrame::OnOffsetUp)
    EVT_TOOL     (ID_OFFSET_DOWN,  DiffFrame::OnOffsetDown)
END_EVENT_TABLE()


class DiffPdfApp : public wxApp
{
public:
    DiffPdfApp() : m_tlw(NULL) {}

    virtual bool OnInit()
    {
        m_tlw = new DiffFrame(m_title);

        // like in LMI, maximize the window
        m_tlw->Maximize();
        m_tlw->Show();

        // yield so that size changes above take effect immediately (and so we
        // can query the window for its size)
        Yield();

        return true;
    }

    void SetData(const wxString& file1, PopplerDocument *doc1,
                 const wxString& file2, PopplerDocument *doc2)
    {
        m_title = wxString::Format("Differences between %s and %s", file1.c_str(), file2.c_str());
        m_doc1 = doc1;
        m_doc2 = doc2;
    }

protected:
    virtual void OnEventLoopEnter(wxEventLoopBase *loop)
    {
        wxApp::OnEventLoopEnter(loop);

        if ( loop->IsMain() )
            SetFrameDocs();
    }

    void SetFrameDocs()
    {
        wxASSERT( m_tlw );
        wxASSERT( m_doc1 );
        wxASSERT( m_doc2 );

        m_tlw->SetDocs(m_doc1, m_doc2);
    }

private:
    DiffFrame *m_tlw;
    wxString m_title;
    PopplerDocument *m_doc1, *m_doc2;
};

IMPLEMENT_APP_NO_MAIN(DiffPdfApp);


// ------------------------------------------------------------------------
// main()
// ------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    wxAppConsole::CheckBuildOptions(WX_BUILD_OPTIONS_SIGNATURE, "diff-pdf");
    wxInitializer wxinitializer(argc, argv);

    static const wxCmdLineEntryDesc cmd_line_desc[] =
    {
        { wxCMD_LINE_SWITCH,
                  "h", "help", "show this help message",
                  wxCMD_LINE_VAL_NONE, wxCMD_LINE_OPTION_HELP },

        { wxCMD_LINE_SWITCH,
                  "v", "verbose", "be verbose" },

        { wxCMD_LINE_SWITCH,
                  "s", "skip-identical", "only output pages with differences" },

        { wxCMD_LINE_SWITCH,
                  "m", "mark-differences", "additionally mark differences on left side" },

        { wxCMD_LINE_SWITCH,
                  "g", "grayscale", "only differences will be in color, unchanged parts will show as gray" },

        { wxCMD_LINE_OPTION,
                  NULL, "output-diff", "output differences to given PDF file",
                  wxCMD_LINE_VAL_STRING },

        { wxCMD_LINE_OPTION,
                  NULL, "output-additions", "output additions to given PDF file",
                  wxCMD_LINE_VAL_STRING },

        { wxCMD_LINE_OPTION,
                  NULL, "output-removals", "output removals to given PDF file",
                  wxCMD_LINE_VAL_STRING },

        { wxCMD_LINE_OPTION,
                  NULL, "channel-tolerance", "consider channel values to be equal if within specified tolerance",
                  wxCMD_LINE_VAL_NUMBER },

        { wxCMD_LINE_OPTION,
                  NULL, "per-page-pixel-tolerance", "total number of pixels allowed to be different per page before specifying the page is different",
                  wxCMD_LINE_VAL_NUMBER },

        { wxCMD_LINE_OPTION,
                  NULL, "shift-tolerance", "check nearby pixels within N pixels distance to ignore small shifts (default: 0, disabled)",
                  wxCMD_LINE_VAL_NUMBER },

        { wxCMD_LINE_SWITCH,
                  NULL, "draw-object-bbox", "draw grayish bounding boxes around each added/removed object in separate diff files" },

        { wxCMD_LINE_SWITCH,
                  NULL, "show-background", "show original PDF as background in additions/removals output files" },

        { wxCMD_LINE_OPTION,
                  NULL, "dpi", "rasterization resolution (default: " wxSTRINGIZE(DEFAULT_RESOLUTION) " dpi)",
                  wxCMD_LINE_VAL_NUMBER },

        { wxCMD_LINE_SWITCH,
                  NULL, "view", "view the differences in a window" },

        { wxCMD_LINE_PARAM,
                  NULL, NULL, "file1.pdf", wxCMD_LINE_VAL_STRING },
        { wxCMD_LINE_PARAM,
                  NULL, NULL, "file2.pdf", wxCMD_LINE_VAL_STRING },

        { wxCMD_LINE_NONE }
    };

    wxCmdLineParser parser(cmd_line_desc, argc, argv);

    switch ( parser.Parse() )
    {
        case -1: // --help
            return 0;

        case 0: // everything is ok; proceed
            break;

        default: // syntax error
            return 2;
    }

    if ( parser.Found("verbose") )
        g_verbose = true;

    if ( parser.Found("skip-identical") )
        g_skip_identical = true;

    if ( parser.Found("mark-differences") )
        g_mark_differences = true;

    if ( parser.Found("grayscale") )
        g_grayscale = true;

    if ( parser.Found("draw-object-bbox") )
        g_draw_object_bbox = true;

    if ( parser.Found("show-background") )
        g_show_background_in_separate = true;

    wxFileName file1(parser.GetParam(0));
    wxFileName file2(parser.GetParam(1));
    file1.MakeAbsolute();
    file2.MakeAbsolute();
    const wxString url1 = wxFileSystem::FileNameToURL(file1);
    const wxString url2 = wxFileSystem::FileNameToURL(file2);

    GError *err = NULL;

    PopplerDocument *doc1 = poppler_document_new_from_file(url1.utf8_str(), NULL, &err);
    if ( !doc1 )
    {
        fprintf(stderr, "Error opening %s: %s\n", (const char*) parser.GetParam(0).c_str(), err->message);
        g_error_free(err);
        return 3;
    }

    PopplerDocument *doc2 = poppler_document_new_from_file(url2.utf8_str(), NULL, &err);
    if ( !doc2 )
    {
        fprintf(stderr, "Error opening %s: %s\n", (const char*) parser.GetParam(1).c_str(), err->message);
        g_error_free(err);
        return 3;
    }

    if ( parser.Found("per-page-pixel-tolerance", &g_per_page_pixel_tolerance) )
    {
        if (g_per_page_pixel_tolerance < 0) {
            fprintf(stderr, "Invalid per-page-pixel-tolerance: %ld. Must be 0 or more\n", g_per_page_pixel_tolerance);
            return 2;
        }
    }

    if ( parser.Found("channel-tolerance", &g_channel_tolerance) )
    {
        if (g_channel_tolerance < 0 || g_channel_tolerance > 255) {
            fprintf(stderr, "Invalid channel-tolerance: %ld. Valid range is 0(default, exact matching)-255\n", g_channel_tolerance);
            return 2;
	}
    }

    if ( parser.Found("shift-tolerance", &g_shift_tolerance) )
    {
        if (g_shift_tolerance < 0 || g_shift_tolerance > 10) {
            fprintf(stderr, "Invalid shift-tolerance: %ld. Valid range is 0(disabled)-10 pixels\n", g_shift_tolerance);
            return 2;
        }
    }

	if ( parser.Found("dpi", &g_resolution) )
    {
        if (g_resolution < 1 || g_resolution > 2400) {
            fprintf(stderr, "Invalid dpi: %ld. Valid range is 1-2400 (default: %d)\n", g_resolution, DEFAULT_RESOLUTION);
            return 2;
	}
    }


    int retval = 0;

    wxString pdf_file;
    wxString pdf_additions;
    wxString pdf_removals;
    
    bool has_output_additions = parser.Found("output-additions", &pdf_additions);
    bool has_output_removals = parser.Found("output-removals", &pdf_removals);
    bool has_output_diff = parser.Found("output-diff", &pdf_file);
    
    if ( has_output_additions || has_output_removals )
    {
        retval = doc_compare_separate(doc1, doc2,
                                     has_output_additions ? pdf_additions.utf8_str() : NULL,
                                     has_output_removals ? pdf_removals.utf8_str() : NULL,
                                     NULL) ? 0 : 1;
    }
    
    if ( has_output_diff )
    {
        int diff_retval = doc_compare(doc1, doc2, pdf_file.utf8_str(), NULL) ? 0 : 1;
        // If we didn't already find differences, use the diff result
        if ( retval == 0 )
            retval = diff_retval;
    }
    else if ( parser.Found("view") )
    {
        wxGetApp().SetData(parser.GetParam(0), doc1,
                           parser.GetParam(1), doc2);
        retval = wxEntry(argc, argv);
    }
    else if ( !has_output_additions && !has_output_removals )
    {
        // No specific output requested, just compare
        retval = doc_compare(doc1, doc2, NULL, NULL) ? 0 : 1;
    }

    g_object_unref(doc1);
    g_object_unref(doc2);

    // MinGW doesn't reliably flush streams on exit, so flush them explicitly:
    fflush(stdout);
    fflush(stderr);

    return retval;
}
