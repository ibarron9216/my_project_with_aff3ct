#include <typeinfo>
#include <algorithm>
#include <map>
#include <thread>
#include <mutex>
#include <vector>

#include "Block.hpp"
#include "Buffered_Socket.hpp"

Block
::Block(aff3ct::module::Task* task, int buffer_size, int n_threads)
: name(task->get_name()),
  n_threads(n_threads),
  tasks(),
  buffer_size(buffer_size),
  threads(n_threads),
  buffered_sockets_in(),
  buffered_sockets_out()
{
	task->set_autoalloc(false);
	task->set_autoexec (false);
	task->set_fast     (false);

	//tasks.push_back(task);
	std::cout << "a" << std::endl;
	for (int i = 0 ; i < n_threads; i++)
		tasks.push_back(task->clone());
		
	std::cout << "b" << std::endl;
	
	int socket_nbr = task->sockets.size();
	for (auto s_idx = 0 ; s_idx < socket_nbr ; s_idx++)
	{
		std::vector<aff3ct::module::Socket* > s_vec;
		for (int i = 0 ; i < n_threads; i++)
			s_vec.push_back(tasks[i]->sockets[s_idx]);
		std::cout << "c" << std::endl;
		aff3ct::module::Socket* s = task->sockets[s_idx];
		std::cout << "d" << std::endl;
		if (task->get_socket_type(*s) == aff3ct::module::Socket_type::IN)
		{
			if(s->get_datatype_string() == "int8")
			{
				this->buffered_sockets_in.emplace(s->get_name(),
				                                  new Buffered_Socket<int8_t>(s_vec, task->get_socket_type(*s), 
				                                  buffer_size));
			}
			else if(s->get_datatype_string() == "int16")
			{
				this->buffered_sockets_in.emplace(s->get_name(),
				                                  new Buffered_Socket<int16_t>(s_vec, task->get_socket_type(*s), 
				                                  buffer_size));
			}
			else if(s->get_datatype_string() == "int32")
			{
				this->buffered_sockets_in.emplace(s->get_name(),
				                                  new Buffered_Socket<int32_t>(s_vec, task->get_socket_type(*s), 
				                                  buffer_size));
			}
			else if(s->get_datatype_string() == "int64")
			{
				this->buffered_sockets_in.emplace(s->get_name(),
				                                  new Buffered_Socket<int64_t>(s_vec, task->get_socket_type(*s), 
				                                  buffer_size));
			}
			else if(s->get_datatype_string() == "float32")
			{
				this->buffered_sockets_in.emplace(s->get_name(),
				                                  new Buffered_Socket<float>(s_vec, task->get_socket_type(*s), 
				                                  buffer_size));
			}
			else if(s->get_datatype_string() == "float64")
			{
				this->buffered_sockets_in.emplace(s->get_name(),
				                                  new Buffered_Socket<double>(s_vec, task->get_socket_type(*s), 
				                                  buffer_size));
			}
		}
		else
		{
			std::cout << "e" << std::endl;
			if(s->get_datatype_string() == "int8")
			{
				std::cout << "e1" << std::endl;
				this->buffered_sockets_out.emplace(s->get_name(),
				                                   new Buffered_Socket<int8_t>(s_vec, task->get_socket_type(*s), 
				                                   buffer_size));
			}
			else if(s->get_datatype_string() == "int16")
			{
				std::cout << "e2" << std::endl;
				this->buffered_sockets_out.emplace(s->get_name(),
				                                   new Buffered_Socket<int16_t>(s_vec, task->get_socket_type(*s), 
				                                   buffer_size));
			}
			else if(s->get_datatype_string() == "int32")
			{
				std::cout << "e3" << std::endl;
				this->buffered_sockets_out.emplace(s->get_name(),
				                                   new Buffered_Socket<int32_t>(s_vec, task->get_socket_type(*s), 
				                                   buffer_size));
				std::cout << "f3" << std::endl;
			}
			else if(s->get_datatype_string() == "int64")
			{
				std::cout << "e4" << std::endl;
				this->buffered_sockets_out.emplace(s->get_name(),
				                                   new Buffered_Socket<int64_t>(s_vec, task->get_socket_type(*s), 
				                                   buffer_size));
			}
			else if(s->get_datatype_string() == "float32")
			{
				std::cout << "e5" << std::endl;
				this->buffered_sockets_out.emplace(s->get_name(),
				                                   new Buffered_Socket<float>(s_vec, task->get_socket_type(*s), 
				                                   buffer_size));
			}
			else if(s->get_datatype_string() == "float64")
			{
				std::cout << "e6" << std::endl;
				this->buffered_sockets_out.emplace(s->get_name(),
				                                   new Buffered_Socket<double>(s_vec, task->get_socket_type(*s), 
				                                   buffer_size));
			}
		}
	}
}

Block
::~Block()
{
	for (auto &it : this->buffered_sockets_in)
		delete it.second;

	for (auto &it : this->buffered_sockets_out)
		delete it.second;
}

int Block
::bind(std::string start_sck_name, Block &dest_block, std::string dest_sck_name)
{
	Buffered_Socket<int8_t>* socket_int8 = this->get_buffered_socket_in<int8_t>(start_sck_name);
	if 	(socket_int8 != nullptr)
		return socket_int8->bind(dest_block.get_buffered_socket_out<int8_t>(dest_sck_name));

	Buffered_Socket<int16_t>* socket_int16 = this->get_buffered_socket_in<int16_t>(start_sck_name);
	if 	(socket_int16 != nullptr)
		return socket_int16->bind(dest_block.get_buffered_socket_out<int16_t>(dest_sck_name));

	Buffered_Socket<int32_t>* socket_int32 = this->get_buffered_socket_in<int32_t>(start_sck_name);
	if 	(socket_int32 != nullptr)
		return socket_int32->bind(dest_block.get_buffered_socket_out<int32_t>(dest_sck_name));

	Buffered_Socket<int64_t>* socket_int64 = this->get_buffered_socket_in<int64_t>(start_sck_name);
	if 	(socket_int64 != nullptr)
		return socket_int64->bind(dest_block.get_buffered_socket_out<int64_t>(dest_sck_name));

	Buffered_Socket<float>* socket_float = this->get_buffered_socket_in<float>(start_sck_name);
	if 	(socket_float != nullptr)
		return socket_float->bind(dest_block.get_buffered_socket_out<float>(dest_sck_name));

	Buffered_Socket<double>* socket_double = this->get_buffered_socket_in<double>(start_sck_name);
	if 	(socket_double != nullptr)
		return socket_double->bind(dest_block.get_buffered_socket_out<double>(dest_sck_name));
	
	std::cout << "No socket named '" << start_sck_name << "' for Task : '" << this->name << "'." << std::endl;
	return 1;	
}

void Block
::run(bool const * isDone) 
{
	for (int i = 0; i < this->n_threads; i++)
		this->threads[i] = std::thread{&Block::execute_task,this,i,isDone};
};

void Block::
join() 
{
	for (auto &th:this->threads)
		th.join();
};

void Block
::execute_task(const int task_id, const bool * isDone)
{
	while(!(*isDone))
	{		
		for (auto const& it : this->buffered_sockets_in  )
			while(!(*isDone) && it.second->pop(task_id)){};

		if (*isDone)
			break;
						
		this->tasks[task_id]->exec();
		for (auto const& it : this->buffered_sockets_out  )
			while(!(*isDone) && it.second->push(task_id)){};
	}	

	for (auto const& it : this->buffered_sockets_out ) { it.second->stop();}
	for (auto const& it : this->buffered_sockets_in ) { it.second->stop();}
}

void Block
::reset()
{
		for (auto const& it : this->buffered_sockets_in  )  { it.second->reset(); }
		for (auto const& it : this->buffered_sockets_out )  { it.second->reset(); }
}

template <typename T>
Buffered_Socket<T>* Block
::get_buffered_socket_in(std::string name)
{
	if (this->buffered_sockets_in.count(name) > 0)
	{	if (aff3ct::module::type_to_string[this->buffered_sockets_in[name]->get_socket()->get_datatype()] == aff3ct::module::type_to_string[typeid(T)])
			return static_cast<Buffered_Socket<T>*>(this->buffered_sockets_in[name]); 
	}
	return nullptr;
};

template <typename T>
Buffered_Socket<T>* Block
::get_buffered_socket_out(std::string name)
{
	if (this->buffered_sockets_out.count(name) > 0)
	{
		if (aff3ct::module::type_to_string[this->buffered_sockets_out[name]->get_socket()->get_datatype()] == aff3ct::module::type_to_string[typeid(T)])
			return static_cast<Buffered_Socket<T>*>(this->buffered_sockets_out[name]); 
	}
	return nullptr;

};