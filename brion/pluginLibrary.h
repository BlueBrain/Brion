/* Copyright (c) 2020, EPFL/Blue Brain Project
 *                     Nadir Román Guerrero <nadir.romanguerrero@epfl.ch>
 *
 * This file is part of Brion <https://github.com/BlueBrain/Brion>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 3.0 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef BRION_PLUGINMANAGER_H
#define BRION_PLUGINMANAGER_H

#include "log.h"
#include "types.h"

#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <typeindex>
#include <unordered_map>

namespace brion
{
namespace
{
/**
 * Plugin factory superclass that allows a container to hold
 * factories of different subclasses of the plugin superclass
 */
template<typename SuperT>
class PluginFactory
{
public:
    using CreateFunc = std::function<std::unique_ptr<SuperT>(const typename SuperT::DataT&)>;
    using HandleFunc = std::function<bool(const typename SuperT::DataT&)>;
    using DescriFunc = std::function<std::string()>;

    PluginFactory(const CreateFunc& createFunc,
                  const HandleFunc& handleFunc,
                  const DescriFunc& descFunc)
     : _createFunc(createFunc)
     , _handleFunc(handleFunc)
     , _descFunc(descFunc)
    {

    }

    bool handles(const typename SuperT::DataT& data) const
    {
        return _handleFunc(data);
    }

    std::string getDescription() const
    {
        return _descFunc();
    }

    std::unique_ptr<SuperT> create(const typename SuperT::DataT& data) const
    {
        return _createFunc(data);
    }

private:
    const CreateFunc _createFunc;
    const HandleFunc _handleFunc;
    const DescriFunc _descFunc;
};

/**
 * Generic function to wrap the plugin creation
 */
template<typename SuperT, typename T>
std::unique_ptr<SuperT> GenericCreateFunc(const typename SuperT::DataT& data)
{
    return std::unique_ptr<SuperT>(new T(data));
}

}

/**
 * @brief The AbstractPluginManager class
 * A superclass for PluginManager that allows to have multiple
 * of them in a stl container
 */
class AbstractPluginManager
{
};

/**
 * Class that manages a bunch of AbstractPluginFactories which produces
 * plugins with the same superclass
 */
template<typename SuperT>
class PluginManager : public AbstractPluginManager
{
public:
    template<typename T>
    void registerFactory()
    {
        std::lock_guard<std::mutex> lock(_mtx);
        _factoryBuffer.push_back(
                    std::make_unique<PluginFactory<SuperT>>(GenericCreateFunc<SuperT, T>,
                                                            T::handles,
                                                            T::getDescription));
    }

    std::unique_ptr<SuperT> create(const typename SuperT::DataT& data) const
    {
        for(const auto& factory : _factoryBuffer)
        {
            if(factory->handles(data))
                return factory->create(data);
        }

        BRION_THROW("Could not find an implementation for " + std::string(typeid(SuperT).name()));
        return {nullptr};
    }

    std::string getDescriptions()
    {
        std::string result ("");
        for(size_t i = 0; i < _factoryBuffer.size() - 1; ++i)
            result += _factoryBuffer[i]->getDescription() + "\n";
        result += _factoryBuffer.back()->getDescription();
        return result;
    }
private:
    std::mutex _mtx;

    using PluginFactoryPtr = std::unique_ptr<PluginFactory<SuperT>>;
    std::vector<PluginFactoryPtr> _factoryBuffer;
};

/**
 * @brief The PluginLibrary class
 * Class that manages an undefined number of plugin managers
 */
class PluginLibrary
{
public:
    static PluginLibrary& instance()
    {
        static PluginLibrary inst;
        return inst;
    }

    template<typename SuperT>
    PluginManager<SuperT>& getManager()
    {
        PluginManager<SuperT>* resultPtr = nullptr;
        auto it = _pluginManagers.find(typeid(SuperT));
        if(it != _pluginManagers.end())
            resultPtr = static_cast<PluginManager<SuperT>*>(it->second.get());
        else
        {
            std::lock_guard<std::mutex> lock(_createMtx);
            it = _pluginManagers.find(typeid(SuperT));
            if(it == _pluginManagers.end())
            {
                std::unique_ptr<AbstractPluginManager> newManager (new PluginManager<SuperT>());
                resultPtr = static_cast<PluginManager<SuperT>*>(newManager.get());
                _pluginManagers[typeid (SuperT)] = std::move(newManager);
            }
            else
                resultPtr = static_cast<PluginManager<SuperT>*>(it->second.get());
        }

        return *resultPtr;
    }

    template<typename SuperT>
    std::unique_ptr<SuperT> create(const typename SuperT::DataT& data)
    {
        PluginManager<SuperT>& manager = getManager<SuperT>();
        return manager.create(data);
    }

private:
    using AbstractManagerPtr = std::unique_ptr<AbstractPluginManager>;
    std::unordered_map<std::type_index, AbstractManagerPtr> _pluginManagers;

    std::mutex _createMtx;
};
}

#endif // PLUGINMANAGER_H

