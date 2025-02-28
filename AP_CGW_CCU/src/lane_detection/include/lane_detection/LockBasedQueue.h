#ifndef LOCKBASEDQUEUE_H_
#define LOCKBASEDQUEUE_H_

#include <mutex>
#include <queue>

#define OUT

template<typename T>
class LockBasedQueue
{
public:
	void Push(T data)
	{
		std::lock_guard<std::recursive_mutex> lock(_rMutex);
		_queue.push(data);
	}

	T Pop()
	{
		std::lock_guard<std::recursive_mutex> lock(_rMutex);
		if (_queue.empty())
			return T();

		T ret = _queue.front();
		_queue.pop();
		return ret;
	}

	void PopAll(OUT std::vector<T>& datas)
	{
		std::lock_guard<std::recursive_mutex> lock(_rMutex);
		while (T data = Pop())
			datas.push_back(data);
	}

	void Clear()
	{
		std::lock_guard<std::recursive_mutex> lock(_rMutex);
		_queue = std::queue<T>();
		//_queue.clear();
	}


private:
	std::recursive_mutex _rMutex;
	std::queue<T> _queue;
};


#endif