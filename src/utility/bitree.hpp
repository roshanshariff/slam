#ifndef _UTILITY_BITREE_HPP
#define _UTILITY_BITREE_HPP

#include <cassert>
#include <vector>
#include <cstddef>

#include <boost/iterator/iterator_facade.hpp>
#include <boost/iterator/iterator_adaptor.hpp>

#include "utility/container_fwd.hpp"


namespace utility {
    
    template <typename Grp, typename Alloc>
    class bitree {
        
    public:
        
        using value_type        = Grp;
        using allocator_type    = Alloc;
        using size_type         = std::size_t;
        using difference_type   = std::ptrdiff_t;
        
        struct const_iterator;
        struct iterator;
        
        struct const_reference {
            friend class bitree;
            friend struct const_iterator;
            operator value_type () const {
                return container->get(index);
            }
        protected:
            const bitree* container;
            const size_type index;
            const_reference (const bitree& c, size_type i) : container(&c), index(i) { }
        };
        

        struct reference : public const_reference {
            friend class bitree;
            friend struct const_iterator;
            const reference& operator= (const value_type& v) const {
                const_cast<bitree*>(const_reference::container)->set(const_reference::index, v);
                return *this;
            }
        protected:
            reference (bitree& c, size_type i) : const_reference(c, i) { }
        };
        
    private:
        
        using const_iterator_base
        = boost::iterator_facade<const_iterator, value_type, boost::random_access_traversal_tag, const_reference>;
        
        using iterator_base
        = boost::iterator_adaptor<iterator, const_iterator, boost::use_default, boost::use_default, reference>;
        
    public:
        
        struct const_iterator : public const_iterator_base {

            friend class boost::iterator_core_access;
            friend struct iterator;
            
            explicit const_iterator (const const_reference& cref) : ref(*cref.container, cref.index) { }

        private:
            
            reference ref;
            
            const const_reference& dereference () const { return ref; }

            void increment () { ++ref.index; }
            void decrement () { --ref.index; }
            void advance (difference_type n) { ref.index += n; }

            difference_type distance_to (const const_iterator& i) const {
                return i.ref.index - ref.index;
            }

            bool equal (const const_iterator& i) const {
                return ref.container == i.ref.container && ref.index == i.ref.index;
            }
        };
        

        struct iterator : public iterator_base {
            
            friend class boost::iterator_core_access;
            
            explicit iterator (const reference& ref) : iterator_base(const_iterator(ref)) { }
            
            operator const const_iterator& () const { return iterator_base::base_reference(); }
            operator const_iterator& () { return iterator_base::base_reference(); }
            
        private:
            
            const reference& dereference () const { return iterator_base::base_reference().ref; }
        };
        
        
        bitree () { }
        bitree (size_type n) : elements(n) { }
        
        
        size_type size () const { return elements.size(); }
        size_type capacity () const { return elements.capacity(); }
        size_type max_size () const { return elements.max_size(); }
        bool empty () const { return elements.empty(); }
        
        reference operator[] (size_type i) { return reference (*this, i); }
        const_reference operator[] (size_type i) const { return const_reference(*this, i); }
        
        reference front () { return (*this)[0]; }
        const_reference front () const { return *this[0]; }

        reference back () { return (*this)[size()-1]; }
        const_reference back () const { return *this[size()-1]; }
        
        iterator begin () { return iterator((*this)[0]); }
        const_iterator begin () const { return const_iterator((*this)[0]); }

        iterator end () { return iterator((*this)[size()]); }
        const_iterator end () const { return const_iterator((*this)[size()]); }

        void pop_back () { elements.pop_back(); }
        void push_back (const value_type& value) { elements.push_back (compute_element(size(), value)); }
        
        void resize (size_type n, const value_type& v = value_type());
        void reserve (size_type n) { elements.reserve(n); }
        void clear () { elements.clear(); }
        
        void swap (bitree& o) { elements.swap (o.elements); } 
        
        value_type accumulate (size_type begin, size_type end) const;
        value_type accumulate (size_type end) const { return accumulate(0, end); }
        value_type accumulate () const { return accumulate(size()); }
        size_type binary_search (const value_type& value) const;
        
    private:
        
        std::vector<value_type, allocator_type> elements;
        
        value_type accumulate_relative (size_type i, size_type ancestor) const;
        value_type compute_element (size_type i, const value_type& value) const;
        void set (size_type i, const value_type& value);
        value_type get (size_type i) const;        
    };
    
    
    namespace bitree_impl {
        
        /** Clears the lowest set bit. */
        inline std::size_t parent (std::size_t i) {
            assert (i > 0);
            return i & (i - 1);
        }
        
        /** Isolates the lowest set bit and adds resulting value to input. */
        inline std::size_t next_sibling (std::size_t i) {
            assert (i > 0);
            return i + (i & -i);
        }
        
        /** Returns the immediate child of 'parent' that contains descendant 'i'. */
        inline std::size_t child_containing (std::size_t parent, std::size_t i) {
            assert (i > parent);
            i -= parent;
            i |= i >> 1;
            i |= i >> 2;
            i |= i >> 4;
            i |= i >> 8;
            i |= i >> 16;
            i |= i >> 32;
            i ^= i >> 1;
            i += parent;
            return i;
        }
        
        /** Returns the closest common ancestor of both a and b. **/
        inline std::size_t common_ancestor (std::size_t a, std::size_t b) {
            b ^= a;
            b |= b >> 1;
            b |= b >> 2;
            b |= b >> 4;
            b |= b >> 8;
            b |= b >> 16;
            b |= b >> 32;
            a &= ~b;
            return a;
        }
        
    }
    
    
    template <typename Grp, typename Alloc>
    auto bitree<Grp, Alloc>
    ::accumulate_relative (size_type i, size_type ancestor) const -> value_type {
        using namespace bitree_impl;
        assert (i < elements.size());
        value_type result = value_type();
        for (; i != ancestor; i = parent(i)) {
            assert (i > 0);
            result = elements[i] + result;
        }
        return result;
    }
    
    
    template <typename Grp, typename Alloc>
    auto bitree<Grp, Alloc>
    ::compute_element (size_type i, const value_type& value) const -> value_type {
        using namespace bitree_impl;
        assert (i <= elements.size());
        if (i == 0) return value;
        else return accumulate_relative(i-1, parent(i)) + value;
    }
    
    
    template <typename Grp, typename Alloc>
    void bitree<Grp, Alloc>
    ::set (size_type i, const value_type& value) {
        using namespace bitree_impl;
        assert (i < elements.size());
        value_type origin = elements[i];
        elements[i] = compute_element(i, value);
        origin += -elements[i];
        if (i > 0) {
            size_t next = next_sibling(i);
            for (; next < elements.size(); i = next, next = next_sibling(i)) {
                size_t next_parent = parent(next);
                while ((i = parent(i)) != next_parent) {
                    origin = elements[i] + origin + (-elements[i]);
                }
                value_type old_value = elements[next];
                elements[next] = -origin + elements[next];
                origin = old_value + (-elements[next]);
            }
        }
    }
    
    
    template <typename Grp, typename Alloc>
    auto bitree<Grp, Alloc>
    ::get (size_type i) const -> value_type {
        using namespace bitree_impl;
        assert (i < elements.size());
        if (i == 0) return elements[0];
        else return -accumulate_relative(i-1, parent(i)) + elements[i];
    }
    
    
    template <typename Grp, typename Alloc>
    void bitree<Grp, Alloc>
    ::resize (size_type n, const value_type& v) {
        if (elements.size() > n) elements.resize(n);
        else while (elements.size() < n) elements.push_back (v);
    }
    
    
    template <typename Grp, typename Alloc>
    auto bitree<Grp, Alloc>
    ::accumulate (size_type begin, size_type end) const -> value_type {
        using namespace bitree_impl;
        assert (begin <= elements.size() && end <= elements.size());
        if (begin == end) {
            return value_type();
        }
        else if (begin == 0) {
            return elements[0] + accumulate_relative(end-1, 0);
        }
        else if (end == 0) {
            return -(elements[0] + accumulate_relative(begin-1, 0));
        }
        else {
            size_type ancestor = common_ancestor(begin-1, end-1);
            return -accumulate_relative(begin-1, ancestor) + accumulate_relative(end-1, ancestor);
        }
    }
    
    
    template <typename Grp, typename Alloc>
    auto bitree<Grp, Alloc>
    ::binary_search (const value_type& value) const -> size_type {
        using namespace bitree_impl;
        if (elements.size() == 0 || value < elements[0]) {
            return 0;
        }
        else {
            size_type min = 0;
            size_type max = elements.size();
            value_type origin = elements[min];
            while (min + 1 < max) {
                size_type mid = child_containing(min, max-1);
                if (value < origin + elements[mid]) {
                    max = mid;
                }
                else {
                    min = mid;
                    origin += elements[min];
                }
            }
            return max;
        }
    }
    
} // namespace utility

#endif //_UTILITY_BITREE_HPP
