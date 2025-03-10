#pragma once

#include "slot.hpp"
#include "ellipsepoint.hpp"
#include "digitalmarkermodel.hpp"

namespace tl {
namespace runetag {

class MarkerDetected
{
    // Friend class allowed to create MarkerDetected instances
    friend class SlotFitter;

private:
    std::vector<Slot> code;

    const DigitalMarkerModel* model;
    cv::Matx33d VR;
    unsigned int num_layers;

    // Creates an instance of MarkerDetected, private factory
    // allowing only SlotFitter to create instances
    static MarkerDetected createDetectedMarker(unsigned int _num_layers)
    {
        MarkerDetected marker;
        marker.offset = 0;
        marker.num_discarded = 0;
        marker.num_errors = 0;
        marker.num_layers = _num_layers;
        return marker;
    }

public:
    class SlotIterator
    {
        friend class MarkerDetected;

    public:
        const Slot& operator*() { return md.code[idx]; }
        const Slot* operator->() { return &(md.code[idx]); }
        void operator++()
        {
            idx += increment;
            if (idx >= md.getNumSlots()) // iteration ended
                idx = -1;
        }
        unsigned int slotIDX() const { return idx; }

        bool operator==(const SlotIterator& it)
        {
            return (&(md) == &(it.md)) && (layer == it.layer) &&
                   (idx == it.idx);
        }
        bool operator!=(const SlotIterator& it) { return !(*this == it); }

    private:
        SlotIterator(const MarkerDetected& _md, int _layer, int start,
                     int _increment)
            : md(_md), layer(_layer), idx(start), increment(_increment) {};
        const MarkerDetected& md;
        const int layer;
        int idx;
        int increment;
    };

    unsigned int offset;
    unsigned int num_errors;
    unsigned int num_discarded;

    SlotIterator begin_by_layer(int layer) const
    {
        return SlotIterator(*this, layer, layer, getNumLayers());
    }
    SlotIterator end_by_layer(int layer) const
    {
        return SlotIterator(*this, layer, -1, getNumLayers());
    }

    SlotIterator begin() const { return SlotIterator(*this, -1, 0, 1); }
    SlotIterator end() const { return SlotIterator(*this, -1, -1, 1); }

    inline unsigned int getNumSymbols() const
    {
        unsigned int n = 0;
        for (std::vector<Slot>::const_iterator it = code.begin();
             it != code.end(); it++) {
            if (it->value()) {
                n++;
            }
        }
        return n;
    }

    inline unsigned int getNumFilledSlots() const
    {
        unsigned int n = 0;
        for (std::vector<Slot>::const_iterator it = code.begin();
             it != code.end(); it++) {
            if (it->getPayload()) {
                n++;
            }
        }
        return n;
    }

    inline std::vector<unsigned int> getNumSymbolsPerLayer() const
    {
        std::vector<unsigned int> spl(getNumLayers(), 0);
        for (size_t l = 0; l < getNumLayers(); ++l) {
            for (SlotIterator it = begin_by_layer(l); it == end_by_layer(l);
                 ++it) {
                if (it->value()) {
                    spl[l]++;
                }
            }
        }
        return spl;
    }

    // Returns the layer with the highest number of filled slots
    inline unsigned int getFullestLayer() const
    {
        unsigned int fullest = 0;
        unsigned int fullval = 0;
        for (size_t l = 0; l < getNumLayers(); ++l) {
            size_t count = 0;
            for (SlotIterator it = begin_by_layer(l); it == end_by_layer(l);
                 ++it) {
                if (it->value()) {
                    count++;
                }
            }
            if (count > fullval) {
                fullval = count;
                fullest = l;
            }
        }
        return fullest;
    }

    inline unsigned int getNumLayers() const { return num_layers; }

    inline void refine(const cv::Mat& gradient_x, const cv::Mat& gradient_y,
                       const cv::Mat& intrinsics)
    {
        throw std::runtime_error("NOT IMPLEMENTED");
        /*
        for( unsigned int i=0; i<code.size(); i++ )
        {
            if( code[i].value() )
                code[i].getPayload()->refine( gradient_x, gradient_y, intrinsics
        );
        }
        */
    }

    inline unsigned int getNumSlots() const
    {
        return (unsigned int)code.size();
    }

    inline const Slot& getSlot(unsigned int index) const
    {
        return code[(index + offset) % code.size()];
    }

    inline const void invalidateSlot(unsigned int index)
    {
        code[(index + offset) % code.size()].invalidate();
    }

    inline bool isExternal(unsigned int index) const
    {
        return (((index + offset) % code.size() + 1) % getNumLayers()) == 0;
    }

    inline const DigitalMarkerModel* associatedModel() const { return model; }

    inline const cv::Matx33d& getVR() const { return VR; }

    inline void associateModel(const DigitalMarkerModel* model,
                               unsigned int offset)
    {
        this->model = model;
        this->offset = offset;
    }
};

} // namespace runetag
} // namespace tl
