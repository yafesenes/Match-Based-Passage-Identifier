#ifndef NARROWPASSAGEREMAKE_RENDERER_H
#define NARROWPASSAGEREMAKE_RENDERER_H

#include "SFML/Graphics.hpp"
#include "Types.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>

using namespace std;

class Line : public sf::Drawable
{
public:
    Line(float p1x, float p1y, float p2x, float p2y, sf::Color color = sf::Color::Black)
            : line {sf::Vertex(sf::Vector2f(p1x, p1y), color), sf::Vertex(sf::Vector2f(p2x, p2y), color)}{}
    sf::Vertex* getLine() { return line; }
private:
    sf::Vertex line[2];

    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(line, 2, sf::Lines, states);
    }
};

class Renderer
{
public:
    static Renderer instance()
    {
        static Renderer renderer;
        return renderer;
    }

    void setMap(Map &map, bool drawGrids = false)
    {
        _Map = map;
        Height = (float)map.size() / (float)map[0].size() * Width;
        pixelSize = (float)Width / (float)(_Map[0].size());
        this->drawGrids = drawGrids;
        _passageValues.resize(map.size(), std::vector<float>(map[0].size(), map.size()));
    }

    void run()
    {
        if (Height == 0)
        {
            std::cerr << "Height is 0" << std::endl;
            return;
        }
        Window = new sf::RenderWindow(sf::VideoMode(Width, Height), "Renderer");
        while (Window->isOpen())
        {
            sf::Event event;
            while (Window->pollEvent(event))
            {
                if (event.type == sf::Event::Closed)
                    Window->close();
            }

            Window->clear();
            Draw();
            DrawPoints();
            DrawGrids();
            Window->display();
        }
    }

    void drawPoints(vector<Point> &points)
    {
        this->points.push_back(points);
    }

    void drawMatches(PassageValues& passageValues)
    {
        _passageValues = passageValues;
    }

    void drawMatch(Point p1, Point p2, sf::Color color = sf::Color::Red)
    {
        lines.push_back({pixelSize*p1.x + pixelSize/2, pixelSize * p1.y + pixelSize/2, pixelSize * p2.x + pixelSize/2, pixelSize*p2.y + pixelSize/2, color});
    }

private:
    void Draw()
    {
        for (int i = 0; i < _Map.size(); i++)
        {
            for (int j = 0; j < _Map[0].size(); j++)
            {
                sf::RectangleShape rect(sf::Vector2f(pixelSize, pixelSize));
                rect.setPosition(sf::Vector2f(pixelSize * j, pixelSize * i));

                if (_Map[i][j]==1)
                {
                    rect.setFillColor(sf::Color(200,200,200));
                    Window->draw(rect);
                }

                else
                {
                    float normalizedValue = _passageValues[i][j] / min(_Map.size(),_Map[0].size());
                    if (normalizedValue > 1)
                        continue;
                    rect.setFillColor(getColorJet(1-normalizedValue));
                    Window->draw(rect);
                }
            }
        }
        for (auto &line : lines)
            Window->draw(line);
    }
    void DrawPoints()
    {
        for (int i = 0; i < points.size(); i++)
            for (const Point &p : points[i])
            {
                sf::CircleShape circle;
                circle.setRadius(pixelSize);
                circle.setPosition(p.x*pixelSize-circle.getRadius()+pixelSize/2, p.y*pixelSize-circle.getRadius()+pixelSize/2);
                circle.setFillColor(sf::Color(getColorJet((float)(i+1) / (float)(points.size()+1))));
                Window->draw(circle);
            }
    }
    void DrawGrids()
    {
        if (drawGrids)
        {
            for (int i = 0; i < _Map.size(); i++)
            {
                for (int j = 0; j < _Map[0].size(); j++)
                {
                    Line vLine(pixelSize * j, 0, pixelSize * j, Height);
                    Line hLine(0, pixelSize*i, Width, pixelSize*i);
                    Window->draw(vLine);
                    Window->draw(hLine);
                }
            }
        }
    }
    sf::Color getColorJet(float value)
    {
        float r, g, b;

        if (value < 0.125f)
        {
            r = 0.0f;
            g = 0.0f;
            b = 4.0f * (value + 0.125f);
        }
        else if (value < 0.375f)
        {
            r = 0.0f;
            g = 4.0f * (value - 0.125f);
            b = 1.0f;
        }
        else if (value < 0.625f)
        {
            r = 4.0f * (value - 0.375f);
            g = 1.0f;
            b = 1.0f - 4.0f * (value - 0.375f);
        }
        else if (value < 0.875f)
        {
            r = 1.0f;
            g = 1.0f - 4.0f * (value - 0.625f);
            b = 0.0f;
        }
        else
        {
            r = 1.0f - 4.0f * (value - 0.875f);
            g = 0.0f;
            b = 0.0f;
        }

        unsigned char red = static_cast<unsigned char>(std::round(r * 255.0f));
        unsigned char green = static_cast<unsigned char>(std::round(g * 255.0f));
        unsigned char blue = static_cast<unsigned char>(std::round(b * 255.0f));

        return sf::Color(red, green, blue);
    }

private:
    const unsigned int Width = 900;
    unsigned int Height;
    sf::RenderWindow* Window;
    float pixelSize;
    bool drawGrids;

    Map _Map;
    vector<Line> lines;
    vector<vector<Point>> points;
    PassageValues _passageValues;
};

#endif //NARROWPASSAGEREMAKE_RENDERER_H
