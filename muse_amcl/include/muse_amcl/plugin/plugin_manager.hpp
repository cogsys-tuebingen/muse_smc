#pragma once

/// SYSTEM
#include <pluginlib/class_loader.h>
#include <class_loader/class_loader.h>
#include <set>
#include <tinyxml.h>
#include <mutex>
#include <functional>
#include <typeindex>

namespace muse_amcl
{

template <class M>
class PluginManagerImp
{
    template <class>
    friend class PluginManager;

protected:
    typedef std::function<std::shared_ptr<M>()> PluginConstructorM;
    typedef std::map<std::string, PluginConstructorM> Constructors;

protected:
    PluginManagerImp(const std::string& full_name)
        : plugins_loaded_(false), full_name_(full_name), loader_("muse_amcl", full_name)
    {
    }

    PluginManagerImp(const PluginManagerImp& rhs);
    PluginManagerImp& operator = (const PluginManagerImp& rhs);

public:
    virtual ~PluginManagerImp() {
    }

protected:

    void load() {
        std::vector<std::string> xml_files = loader_.getPluginXmlPaths();

        for(std::vector<std::string>::const_iterator manifest = xml_files.begin(); manifest != xml_files.end(); ++manifest) {
            processManifest(*manifest);
        }

        plugins_loaded_ = true;
    }

    bool processManifest(const std::string& xml_file)
    {
        TiXmlDocument document;
        document.LoadFile(xml_file);
        TiXmlElement * config = document.RootElement();
        if (config == nullptr) {
            std::cerr << "[Plugin] Cannot load the file " << xml_file << std::endl;
            return false;
        }
        if (config->ValueStr() != "library") {
            std::cerr << "[Plugin] Manifest root is not <library>" << std::endl;
            return false;
        }

        TiXmlElement* library = config;
        while (library != nullptr) {

            std::string library_name = library->Attribute("path");
            if (library_name.size() == 0) {
                std::cerr << "[Plugin] Item in row" << library->Row() << " does not contain a path attribute" << std::endl;
                continue;
            }

            loadLibrary(library_name, library);

            library = library->NextSiblingElement( "library" );
        }

        return true;
    }

    std::string loadLibrary(const std::string& library_name, TiXmlElement* library)  {
        std::string library_path = library_name + ".so";

        bool load = false;
        {
            TiXmlElement* class_element = library->FirstChildElement("class");
            while (class_element) {
                std::string base_class_type = class_element->Attribute("base_class_type");
                if(base_class_type == full_name_) {
                    load = true;
                    break;
                }
                class_element = class_element->NextSiblingElement( "class" );
            }
        }

        if(!load) {
            return "";
        }

        std::shared_ptr<class_loader::ClassLoader> loader(new class_loader::ClassLoader(library_path));

        TiXmlElement* class_element = library->FirstChildElement("class");
        while (class_element) {
            loadClass(library_name, class_element, loader.get());

            class_element = class_element->NextSiblingElement( "class" );
        }
        loaders_[library_name] = std::move(loader);

        return library_path;
    }

    void loadClass(const std::string& library_name, TiXmlElement* class_element, class_loader::ClassLoader* loader) {
        std::string base_class_type = class_element->Attribute("base_class_type");
        std::string derived_class = class_element->Attribute("type");

        std::string lookup_name;
        if(class_element->Attribute("name") != nullptr) {
            lookup_name = class_element->Attribute("name");
        } else {
            lookup_name = derived_class;
        }

        if(base_class_type == full_name_){
            std::string description = readString(class_element, "description");
            std::string icon = readString(class_element, "icon");
            std::string tags = readString(class_element, "tags");

            available_classes.emplace(lookup_name, [loader, lookup_name]() {
                return std::shared_ptr<M> { loader->createUnmanagedInstance<M>(lookup_name) };
            });
        }
    }

    std::string readString(TiXmlElement* class_element, const std::string& name) {
        TiXmlElement* description = class_element->FirstChildElement(name);
        std::string description_str;
        if(description) {
            description_str = description->GetText() ? description->GetText() : "";
        }

        return description_str;
    }

protected:
    bool plugins_loaded_;
    std::string full_name_;

    pluginlib::ClassLoader<M> loader_;
    std::map< std::string, std::shared_ptr<class_loader::ClassLoader>> loaders_;

    Constructors available_classes;
};

class PluginManagerLocker
{
public:
    static std::mutex& getMutex()
    {
        static PluginManagerLocker instance;
        return instance.mutex;
    }

private:
    PluginManagerLocker()
    {}

private:
    std::mutex mutex;
};

template <class M>
class PluginManager
{
protected:
    typedef PluginManagerImp<M> Parent;

public:
    typedef typename Parent::PluginConstructorM Constructor;
    typedef typename Parent::Constructors Constructors;

    PluginManager(const std::string& full_name)
    {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        if(i_count == 0) {
            ++i_count;
            instance = new Parent(full_name);
        }
    }

    virtual ~PluginManager()
    {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        --i_count;
        if(i_count == 0) {
            delete instance;
        }
    }

    virtual bool pluginsLoaded() const {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        return instance->plugins_loaded_;
    }

    virtual void load() {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());

        instance->load();
    }

    Constructor getConstructor(const std::string& name) {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        auto pos = instance->available_classes.find(name);

        if(pos != instance->available_classes.end()) {
            return pos->second;
        } else {
            return {};
        }
    }

protected:
    static int i_count;
    static Parent* instance;
};

template <class M>
int PluginManager<M>::i_count = 0;
template <class M>
typename PluginManager<M>::Parent* PluginManager<M>::instance(nullptr);

}
