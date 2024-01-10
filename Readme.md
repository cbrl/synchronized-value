# Synchronized Value

The manual association of a mutex with a value in standard C++ is both unergonomic and prone to errors. For example,
locking an incorrent mutex to access a value, or failing to lock the mutex at all. The compiler will not provide any
diagnostics for these errors.

A common pattern is to create a class which associates a mutex with a value. This both makes it harder to incorrectly
use the mutex and cleans up the code surrounding the synchronization points. This project provides an implementation of
this design pattern.

```cpp
auto value = synchronized_value<std::string>{};

// Set value
value = "Example1";
value.set("Example2");
value.try_set_for("Example3", std::chrono::seconds{1});
value.with_write_lock([](std::string& str) { str = "Example4"; });
{
	auto locked_value = value.lock_write();
	*locked_value = "Example5";
}

// Retrieve value
std::cout << value.get() << std::endl;
std::cout << value.try_get_for(std::chrono::seconds{1}).value_or("Failure") << std::endl;
value.with_read_lock([](std::string const& str) { std::cout << str << std::endl; });
{
	auto locked_value = value.lock_read();
	std::cout << *locked_value << std::endl;
}
```
