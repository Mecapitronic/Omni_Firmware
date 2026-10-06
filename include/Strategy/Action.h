#ifndef ACTION_H
#define ACTION_H

#include "ESP32_Helper.h"
#include "Structure.h"
#include <cmath>
#include <vector>

struct Action : public IJsonSerializable
{
    uint16_t ID = 0;
    t_vertexID vertexID = 0;
    int8_t mission = 0;
    bool possible = false;
    bool done = false;
    uint8_t iteration = 0;
    Point point;
    String function = "";

    uint16_t Version() const override { return 1; }

    bool FromJson(const JsonDocument &document) override
    {
        return FromJson(document.as<JsonObjectConst>());
    }

    bool FromJson(JsonObjectConst object)
    {
        if (!object["ID"].is<uint16_t>() ||
            !object["vertexID"].is<String>() ||
            !object["mission"].is<int8_t>() ||
            !object["possible"].is<bool>() ||
            !object["done"].is<bool>() ||
            !object["iteration"].is<uint8_t>() ||
            !object["point"].is<JsonObjectConst>() ||
            !object["point"]["x"].is<double>() ||
            !object["point"]["y"].is<double>() ||
            !object["function"].is<String>())
        {
            Printer::println("Invalid action JSON: expected ID, vertexID, mission, possible, done, iteration, point {x, y}, function");
            return false;
        }

        const String vertexText = object["vertexID"].as<String>();
        uint16_t vertexValue = 0;
        if (vertexText.isEmpty())
            return false;
        for (size_t index = 0; index < vertexText.length(); ++index)
        {
            const char digit = vertexText[index];
            if (digit < '0' || digit > '9')
                return false;
            vertexValue = vertexValue * 10 + (digit - '0');
            if (vertexValue > UINT8_MAX)
                return false;
        }

        const double pointX = object["point"]["x"].as<double>();
        const double pointY = object["point"]["y"].as<double>();
        if (!std::isfinite(pointX) || !std::isfinite(pointY) ||
            pointX < INT16_MIN || pointX > INT16_MAX ||
            pointY < INT16_MIN || pointY > INT16_MAX ||
            std::trunc(pointX) != pointX || std::trunc(pointY) != pointY)
        {
            Printer::println("Invalid point: x and y must be integral int16_t coordinates");
            return false;
        }

        ID = object["ID"].as<uint16_t>();
        vertexID = static_cast<t_vertexID>(vertexValue);
        mission = object["mission"].as<int8_t>();
        possible = object["possible"].as<bool>();
        done = object["done"].as<bool>();
        iteration = object["iteration"].as<uint8_t>();
        point = Point(static_cast<int16_t>(pointX), static_cast<int16_t>(pointY));
        function = object["function"].as<String>();

        return true;
    }

    void ToJson(JsonDocument &document) const override
    {
        ToJson(document.to<JsonObject>());
    }

    void ToJson(JsonObject object) const
    {
        object["ID"] = ID;
        object["vertexID"] = String(vertexID);
        object["mission"] = mission;
        object["possible"] = possible;
        object["done"] = done;
        object["iteration"] = iteration;
        object["point"]["x"] = point.x;
        object["point"]["y"] = point.y;
        object["function"] = function;
    }

    void Print() const
    {
        Printer::println("ID = %u, vertexID = %u, mission = %d, possible = %s, done = %s, iteration = %u, point = (%d, %d), function = %s",
                         static_cast<unsigned>(ID), static_cast<unsigned>(vertexID), static_cast<int>(mission),
                         possible ? "true" : "false", done ? "true" : "false", static_cast<unsigned>(iteration),
                         static_cast<int>(point.x), static_cast<int>(point.y), function.c_str());
    }
};

struct Actions : public IJsonSerializable
{
    String date;
    std::vector<Action> actions;

    uint16_t Version() const override { return 1; }

    bool FromJson(const JsonDocument &document) override
    {
        if (!document["version"].is<uint16_t>() || document["version"].as<uint16_t>() != Version() ||
            !document["date"].is<String>() || !document["actions"].is<JsonArrayConst>())
            return false;

        std::vector<Action> loadedActions;
        for (JsonObjectConst object : document["actions"].as<JsonArrayConst>())
        {
            Action action;
            if (!action.FromJson(object))
                return false;
            loadedActions.push_back(action);
        }
        date = document["date"].as<String>();
        actions.swap(loadedActions);
        return true;
    }

    void ToJson(JsonDocument &document) const override
    {
        document.to<JsonObject>();
        document["version"] = Version();
        document["date"] = date;
        JsonArray array = document["actions"].to<JsonArray>();
        for (const Action &action : actions)
            action.ToJson(array.add<JsonObject>());
    }
};

#endif
