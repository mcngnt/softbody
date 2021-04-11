#include <SFML/Graphics.hpp>
#include "softbody.h"


int main()
{
	std::srand(static_cast<unsigned int>(std::time(NULL)));
    sf::RenderWindow window(sf::VideoMode(SCREEN_W, SCREEN_H, 32), "Main", sf::Style::Default);
    sf::RenderTexture targetTex;
    targetTex.create(SCREEN_W, SCREEN_H);
    targetTex.setSmooth(true);
    sf::Sprite targetSprite;
    targetSprite.setTexture(targetTex.getTexture());
    targetSprite.setOrigin(sf::Vector2f(targetSprite.getTexture()->getSize().x * targetSprite.getScale().x,targetSprite.getTexture()->getSize().y * targetSprite.getScale().y)/2.f);
    targetSprite.setPosition(SCREEN_W/2, SCREEN_H/2);
    targetSprite.setScale(1.0, -1.0);
    // window.setVerticalSyncEnabled(true);

    float dt = 0.2;
    int tick = 0;
    float time = 0.f;
    float oldDT = dt;
    bool isPaused = false;
    sf::Vector2f mousePos(0, 0);
    bool isHoldingLeft = false;
    SoftbodyManager sMan(vec_v(40, 22.5), 0.0005);
    
    std::vector<Vector> firstRect = {vec_v(-4, 8),  vec_v(4, 5), vec_v(2, 4), vec_v(-5, 7)};
    sMan.add_shape(firstRect);
    std::vector<Vector> secondRect = {vec_v(14, 5), vec_v(16, 4), vec_v(8, 1), vec_v(6, 2)};
    sMan.add_shape(secondRect);
    std::vector<Vector> thirdRect = {vec_v(-10, -2), vec_v(10, -2), vec_v(10, -5), vec_v(-10, -5)};
    sMan.add_shape(thirdRect);
    std::vector<Vector> triangle = {vec_v(-5, -2.01), vec_v(0, -2.01), vec_v(-2.5, 1)};
    sMan.add_shape(triangle);
    sMan.add_shape(Shape(vec_v(10, -3), 3));
    sMan.add_body(Softbody(vec_v(0, 10), .2, 0.65f, 0.3, 1));

    std::vector<Vector> userShape;
    

    while(window.isOpen())
    {
    	mousePos = sf::Vector2f(sf::Mouse::getPosition(window));
        sf::Event event;
        while (window.pollEvent(event))
        {
            if ((event.type == sf::Event::Closed) || ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Escape)))
            {
                window.close();
                break;
            }

            if (event.type == sf::Event::KeyPressed)
            {
                if (event.key.code == sf::Keyboard::Up)
                {
                    dt *= 2;
                }
                if (event.key.code == sf::Keyboard::Down)
                {
                    dt /= 2;
                }
                if (event.key.code == sf::Keyboard::R)
                {
                    if (sMan.bodies.size() > 0)
                    {
                        sMan.bodies.erase(sMan.bodies.begin());
                    }
                    sMan.add_body(Softbody(vec_v(0, 10), .1, 0.65f, 0.3, 1));
                }
                if (event.key.code == sf::Keyboard::Space)
                {
                    isPaused = !isPaused;
                    if (isPaused)
                    {
                        oldDT = dt;
                        dt = 0;
                    }
                    else
                    {
                        dt = oldDT;
                    }
                }
                if (event.key.code == sf::Keyboard::T)
                {
                    if (userShape.size() > 2)
                    {
                        sMan.add_shape(userShape);
                        userShape.clear();
                    }
                    if (userShape.size() == 2)
                    {
                        sMan.add_shape(Shape(userShape[0], dist_v(userShape[0], userShape[1])));
                        userShape.clear();
                    }
                }
            }

            if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    isHoldingLeft = true;
                }
                if (event.mouseButton.button == sf::Mouse::Right)
                {
                    userShape.push_back(sMan.screen_to_world_pos(sfvec2_to_vec_v(mousePos)));
                }
                if (event.mouseButton.button == sf::Mouse::Middle)
                {
                    bool hasDestroyedShape = false;
                    for (int i = 0; i < sMan.shapes.size(); ++i)
                    {
                        Shape myShape = sMan.shapes[i];
                        if (myShape.shapeType == SHAPE_POLYGON)
                        {
                            if (dist_v(sMan.screen_to_world_pos(sfvec2_to_vec_v(mousePos)), myShape.polygonMiddlePos) < sMan.worldSize.x/10.f)
                            {
                                sMan.shapes.erase(sMan.shapes.begin() + i);
                                hasDestroyedShape = true;
                            }
                        }
                        if (myShape.shapeType == SHAPE_CIRCLE)
                        {
                            if (dist_v(sMan.screen_to_world_pos(sfvec2_to_vec_v(mousePos)), myShape.circlePos) < sMan.worldSize.x/10.f)
                            {
                                sMan.shapes.erase(sMan.shapes.begin() + i);
                                hasDestroyedShape = true;
                            }
                        }
                        if (hasDestroyedShape)
                        {
                            break;
                        }
                    }
                }
            }
            if (event.type == sf::Event::MouseButtonReleased)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    isHoldingLeft = false;
                }
            }
        }

        tick++;
        time += dt;
        targetTex.clear(sf::Color(30, 30, 50, 255));
        sMan.update_bodies(dt);
        if (isHoldingLeft)
        {
            Vector worldMousePos = sMan.screen_to_world_pos(sfvec2_to_vec_v(mousePos));
            if (sMan.bodies.size() > 0)
            {
                for (int i = 0; i < sMan.bodies[0].points.size(); ++i)
                {
                    sMan.bodies[0].points[i].vel += normalize_v(worldMousePos - sMan.bodies[0].points[i].pos) * 0.0002;

                }
            }
            
        }
        sf::VertexArray line(sf::Lines, 2);
        sf::CircleShape circle;

        for (unsigned i = 0; i < sMan.shapes.size(); ++i)
        {
            Shape myShape = sMan.shapes[i];
            if (myShape.shapeType == SHAPE_CIRCLE)
            {
                circle.setRadius(myShape.circleRadius/sMan.worldSize.x * SCREEN_W);
                circle.setOrigin(circle.getRadius(), circle.getRadius());
                circle.setPosition(vec_to_sfvec2_v(sMan.world_to_screen_pos(myShape.circlePos)));
                circle.setOutlineColor(sf::Color::White);
                circle.setOutlineThickness(2);
                circle.setFillColor(sf::Color::Transparent);
                targetTex.draw(circle);

            }
            if (myShape.shapeType == SHAPE_POLYGON)
            {
                sf::ConvexShape convex;
                convex.setPointCount(myShape.vertices.size());
                for (unsigned j = 0; j < myShape.vertices.size(); ++j)
                {
                    convex.setPoint(j, vec_to_sfvec2_v(sMan.world_to_screen_pos(myShape.vertices[j])));
                }
                convex.setOutlineColor(sf::Color::White);
                convex.setOutlineThickness(2);
                convex.setFillColor(sf::Color::Transparent);
                targetTex.draw(convex);
            }
        }

        for (unsigned i = 0; i < sMan.bodies.size(); ++i)
        {
            Softbody* myBody = &sMan.bodies[i];
            for (unsigned j = 0; j < myBody->springs.size(); ++j)
            {
                line[0].position = vec_to_sfvec2_v(sMan.world_to_screen_pos(myBody->points[myBody->springs[j].aID].pos));
                line[1].position = vec_to_sfvec2_v(sMan.world_to_screen_pos(myBody->points[myBody->springs[j].bID].pos));
                targetTex.draw(line);
            }

            circle.setOutlineColor(sf::Color::Transparent);
            for (unsigned j = 0; j < myBody->points.size(); ++j)
            {
                float ratio = (float)j/(float)myBody->points.size();
                circle.setFillColor(sf::Color(ratio * 255, (1.f - ratio) * 255, (ratio/2.f)*255, 255));
                Point* myPoint = &myBody->points[j];
                circle.setRadius(myPoint->radius/sMan.worldSize.x * SCREEN_W);
                circle.setOrigin(circle.getRadius(), circle.getRadius());
                circle.setPosition(vec_to_sfvec2_v(sMan.world_to_screen_pos(myPoint->pos)));
                targetTex.draw(circle);
            }

        }

        if (userShape.size() == 2)
        {
            Shape myShape(userShape[0], dist_v(userShape[0], userShape[1]));
            circle.setRadius(myShape.circleRadius/sMan.worldSize.x * SCREEN_W);
            circle.setOrigin(circle.getRadius(), circle.getRadius());
            circle.setPosition(vec_to_sfvec2_v(sMan.world_to_screen_pos(myShape.circlePos)));
            circle.setOutlineColor(sf::Color::Yellow);
            circle.setOutlineThickness(2);
            circle.setFillColor(sf::Color::Transparent);
            targetTex.draw(circle);
        }
        if (userShape.size() > 2)
        {
            Shape myShape(userShape);
            sf::ConvexShape convex;
            convex.setPointCount(myShape.vertices.size());
            for (unsigned j = 0; j < myShape.vertices.size(); ++j)
            {
                convex.setPoint(j, vec_to_sfvec2_v(sMan.world_to_screen_pos(myShape.vertices[j])));
            }
            convex.setOutlineColor(sf::Color::Yellow);
            convex.setOutlineThickness(2);
            convex.setFillColor(sf::Color::Transparent);
            targetTex.draw(convex);
        }

        window.draw(targetSprite);
        window.display();

    }

	return 0;
}