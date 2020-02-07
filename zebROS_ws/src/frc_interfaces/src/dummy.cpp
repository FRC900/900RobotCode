// For header-only packages, create a dummy c++ file
// to force cmake to export compile_commands.json
// This file is required for YCM to understand how to
// parse files in this package
int main (void)
{
	return 0;
}
