#include "grid/updatable_priority_queue.h"

template <typename Key, typename Priority>
bool updatable_priority_queue<Key, Priority>::empty()        { return heap.empty(); }

template <typename Key, typename Priority>
std::size_t updatable_priority_queue<Key, Priority>::size()  { return heap.size(); }

/** first is priority, second is key */
template <typename Key, typename Priority>
const priority_queue_node<Key,Priority>& updatable_priority_queue<Key, Priority>::top() { return heap.front(); }

template <typename Key, typename Priority>
void updatable_priority_queue<Key, Priority>::pop(bool remember_pop) { //pop=false
  if(size() == 0) return;
  id_to_heappos[heap.front().key] = -1-remember_pop;
  if(size() > 1)
    *heap.begin() = std::move(*(heap.end()-1));
  heap.pop_back();
  sift_down(0);
}

template <typename Key, typename Priority>
Key updatable_priority_queue<Key, Priority>::pop_value(bool remember_key) { //remember_key=true
  if(size() == 0) return -1;
  priority_queue_node<Key,Priority> ret = std::move(*heap.begin());
  id_to_heappos[ret.key] = -1-remember_key;
  if(size() > 1)
    *heap.begin() = std::move(*(heap.end()-1));
  heap.pop_back();
  sift_down(0);
  return ret.key;
}

/** Sets the priority for the given key. If not present, it will be added, otherwise it will be updated
 *  Returns true if the priority was changed.
 * */
 template <typename Key, typename Priority>
bool updatable_priority_queue<Key, Priority>::set(const Key& key, const Priority& priority, bool only_if_higher) { //only_if_higher=false
  if(key < id_to_heappos.size() && id_to_heappos[key] >= 0) // This key is already in the pQ
    return update(key, priority, only_if_higher);
  else
    return push(key, priority, only_if_higher);
}

template <typename Key, typename Priority>
std::pair<bool,Priority> updatable_priority_queue<Key, Priority>::get_priority(const Key& key) {
  if(key >= id_to_heappos.size()) {
    size_t pos = id_to_heappos[key];
    if(pos >= 0) {
      return {true, heap[pos].priority};
    }
  }
  return {false, 0};
}

/** Returns true if the key was not inside and was added, otherwise does nothing and returns false
 *  If the key was remembered and only_if_unknown is true, does nothing and returns false
 * */
 template <typename Key, typename Priority>
bool updatable_priority_queue<Key, Priority>::push(const Key& key, const Priority& priority, bool only_if_unknown) { //only_if_unknown=false
  extend_ids(key);
  if(id_to_heappos[key] < ((size_t)-2)) return false;
  if(only_if_unknown && id_to_heappos[key] == ((size_t)-2)) return false;
  // otherwise we have id_to_heappos[key] = -1, unseen key
  size_t n = heap.size();
  id_to_heappos[key] = n; // For consistency in the case where nothing moves (early return)
  heap.emplace_back(key,priority);
  sift_up(n);
  return true;
}

/** Returns true if the key was already inside and was updated, otherwise does nothing and returns false */
template <typename Key, typename Priority>
bool updatable_priority_queue<Key, Priority>::update(const Key& key, const Priority& new_priority, bool only_if_higher) {  //only_if_higher=false
  if(key >= id_to_heappos.size()){
    push(key, new_priority, false);
    return true;
  }
  size_t heappos = id_to_heappos[key];
  if(heappos >= ((size_t)-2)){
    push(key, new_priority,false);
    return true;
  }
  Priority& priority = heap[heappos].priority;
  if(new_priority < priority) {
    priority = new_priority;
    sift_up(heappos);
    return true;
  }
  else if(!only_if_higher && new_priority > priority) {
    priority = new_priority;
    sift_down(heappos);
    return true;
  }
  return false;
}

template <typename Key, typename Priority>
void updatable_priority_queue<Key, Priority>::extend_ids(Key k) {
  size_t new_size = k+1;
  if(id_to_heappos.size() < new_size)
    id_to_heappos.resize(new_size, -1);
}

template <typename Key, typename Priority>
void updatable_priority_queue<Key, Priority>::sift_down(size_t heappos) {
  size_t len = heap.size();
  size_t child = heappos*2+1;
  if(len < 2 || child >= len) return;
  if(child+1 < len && heap[child+1] < heap[child]) ++child; // Check whether second child is higher
  if(!(heap[child] < heap[heappos])) return; // Already in heap order

  priority_queue_node<Key,Priority> val = std::move(heap[heappos]);
  do {
    heap[heappos] = std::move(heap[child]);
    id_to_heappos[heap[heappos].key] = heappos;
    heappos = child;
    child = 2*child+1;
    if(child >= len) break;
    if(child+1 < len && heap[child+1] < heap[child]) ++child;
  } while(heap[child] < val);
  heap[heappos] = std::move(val);
  id_to_heappos[heap[heappos].key] = heappos;
}

template <typename Key, typename Priority>
void updatable_priority_queue<Key, Priority>::sift_up(size_t heappos) {
  size_t len = heap.size();
  if(len < 2 || heappos <= 0) return;
  size_t parent = (heappos-1)/2;
  if(!(heap[heappos] < heap[parent])) return;
  priority_queue_node<Key, Priority> val = std::move(heap[heappos]);
  do {
    heap[heappos] = std::move(heap[parent]);
    id_to_heappos[heap[heappos].key] = heappos;
    heappos = parent;
    if(heappos <= 0) break;
    parent = (parent-1)/2;
  } while(val < heap[parent]);
  heap[heappos] = std::move(val);
  id_to_heappos[heap[heappos].key] = heappos;
}

template class updatable_priority_queue<int, int>;
