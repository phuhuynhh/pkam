#include <utility>
#include <vector>
#ifndef UPDATABLE_PRIORITY_QUEUE_H
#define UPDATABLE_PRIORITY_QUEUE_H

	template <typename Key, typename Priority>
		struct priority_queue_node {
			Priority priority;
			Key key;
			priority_queue_node(const Key& key, const Priority& priority) : priority(priority), key(key) {}
			friend bool operator<(const priority_queue_node& pqn1, const priority_queue_node& pqn2) {
				return pqn1.priority < pqn2.priority;
			}
			friend bool operator>(const priority_queue_node& pqn1, const priority_queue_node& pqn2) {
				return pqn1.priority > pqn2.priority;
			}
		};

	/** Key has to be an uint value (convertible to size_t)
	 * This is a max heap (max is on top), to match stl's pQ */
	template <typename Key, typename Priority>
		class updatable_priority_queue {
			protected:
				std::vector<size_t> id_to_heappos;
				std::vector<priority_queue_node<Key,Priority>> heap;

			public:
				updatable_priority_queue() {}

				bool empty();
				std::size_t size();

				/** first is priority, second is key */
				const priority_queue_node<Key,Priority>& top();

				void pop(bool remember_); //pop=false

				Key pop_value(bool remember_key); //remember_key=true

				/** Sets the priority for the given key. If not present, it will be added, otherwise it will be updated
				 *  Returns true if the priority was changed.
				 * */
				bool set(const Key& key, const Priority& priority, bool only_if_higher); //only_if_higher=false

				std::pair<bool,Priority> get_priority(const Key& key);

				/** Returns true if the key was not inside and was added, otherwise does nothing and returns false
				 *  If the key was remembered and only_if_unknown is true, does nothing and returns false
				 * */
				bool push(const Key& key, const Priority& priority, bool only_if_unknown);//only_if_unknown=false

				/** Returns true if the key was already inside and was updated, otherwise does nothing and returns false */
				bool update(const Key& key, const Priority& new_priority, bool only_if_higher);  // only_if_higher=false

			private:
				void extend_ids(Key k);

				void sift_down(size_t heappos);

				void sift_up(size_t heappos);
		};

#endif
