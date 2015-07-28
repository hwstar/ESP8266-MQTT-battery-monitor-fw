#include "ets_sys.h"
#include "os_type.h"
#include "mem.h"
#include "osapi.h"
#include "user_interface.h"
#include "jsonparse.h"
#include "util.h"

/*
 * Restart system
 */
 
void ICACHE_FLASH_ATTR util_restart(void)
{
	os_printf("\r\nRestarting...\r\n");
	ets_delay_us(250000);
	system_restart();
}


/*
 * Assert error handler
 */
 

void ICACHE_FLASH_ATTR util_assert_handler(void)
{	
	util_restart();
}



/*
 * util_str_realloc - return a new string pointer in a larger buffer
 */
 
char * ICACHE_FLASH_ATTR util_str_realloc(const char *p, size_t new_len)
{
	size_t oldlen = strlen(p) + 1;
	util_assert(new_len > oldlen + 1, "New string smaller than original string. new length: %d, old length: %d", new_len, oldlen); 
	char *n =  util_zalloc(new_len);
	os_strcpy(n, p);
	util_free(p);
	return n;
}



/*
 * strdup function missing from os_* calls.
 */
 
char * ICACHE_FLASH_ATTR util_strdup(const char *s)
{
	char *p =  util_zalloc(os_strlen(s) + 1);
	os_strcpy(p, s);
	return p;
}



/*
 * strndup function missing from os_* calls.
 */
 
char * ICACHE_FLASH_ATTR util_strndup(const char *s, int len)
{
	char *p =  util_zalloc(len + 1);
	os_strncpy(p, s, len);
	p[len] = 0;
	return p;
}
 




/*
* Split a string into substrings and return the result as a list of pointers
* List of pointers should be 1 more than what is required. 
* max_list_length includes the NULL marker.
* 
* Returned char pointer must be freed when no longer required.
* 
* Does not remove white space
*/
char * ICACHE_FLASH_ATTR util_string_split(const char *in_str, char **list, char sep, int max_list_length)
{
	int i, j;
	char *str = util_strdup(in_str);
	
	for(i = 0, j = 0; i < max_list_length - 1; i++){

		// Skip leading seps
		while(sep == str[j]){
			if(!str[j])
				break;
			j++;
		}
		
		// Test for empty entry
		if(!str[j])
			break;
			
		list[i] = str + j; // Save the beginning of the string
		while(str[j] && (str[j] != sep))
			j++;
		// Test for end of string
		if(!str[j]){
			i++;
			break;
		}
		str[j] = 0; // Terminate substring
		j++;
	}
	list[i] = NULL; // Terminate end of list
	return str;
}

/*
 * Allocate and make a subtopic string 
 */

char * ICACHE_FLASH_ATTR util_make_sub_topic(const char *rootTopic, char *subTopic)
{
	char *r = (char *) os_zalloc(strlen(rootTopic) + 
	2 + strlen(subTopic));
	os_strcpy(r, rootTopic);
	os_strcat(r, "/");
	os_strcat(r, subTopic);
	return r;
}

/*
 * Parse a json parameter from a parsed json string
 * Returns  0 if no name was found or matched.
 * Returns  1 if a name was matched, but no parameter was found
 * Returns 2 if a name was found and matched, and a parameter was found and matched.
 * If 2 is returned, the parameter will be copied into the paramvalue string up to the paramvaluesize
 */

int ICACHE_FLASH_ATTR util_parse_json_param(void *state, const char *paramname, char *paramvalue, int paramvaluesize)
{
	int i = 0;
	int res;
		
	while((res = jsonparse_next((struct jsonparse_state *) state)) != 0){
		if(res == 'N'){
			if(!jsonparse_strcmp_value((struct jsonparse_state *) state, paramname)){
				i++;
			}
		}
		if(i && (res == '"')){
			i++;
			jsonparse_copy_value((struct jsonparse_state *) state, paramvalue, paramvaluesize);
			break;
		}
	}
	return i;
}

/*
 * Local function to to support util_parse_command_int and util_parse_command_qstring
 * Comares command and parses message looking for a parameter field. This can return -1 in addition to
 * 0,1, and 2 from parse_json_param. -1 indicates the command did not match.
 */
 
LOCAL int ICACHE_FLASH_ATTR processCommandParam(const char *commandrcvd, const char *command, const char *message, char *param, int paramsize)
{
	struct jsonparse_state state;
	
	if(strcmp(command, commandrcvd)){
		return -1;
	}
	param[0] = 0;	
	jsonparse_setup(&state, message, os_strlen(message));	
	return util_parse_json_param(&state, "param", param, paramsize);

}


/*
 * Parse a command with a single numeric parameter
 * If the command is not matched, and  the number is not present, FALSE is returned.
 */

bool ICACHE_FLASH_ATTR util_parse_command_int(const char *commandrcvd, const char *command, const char *message, int *val)
{
	int res;
	char param[32];
	struct jsonparse_state state;
	
	res = processCommandParam(commandrcvd, command, message, param, sizeof(param));

	if(2 == res){
		*val = atoi(param);
		return TRUE;
	}
	return FALSE;
}

/*
 * Parse a command with an optional string parameter
 * Returns FALSE if the command is not matched.
 * Sets val to NULL if no parameter found, else returns an allocated string pointing to the
 * value. This string must be freed when no longer required.
 */
 
bool ICACHE_FLASH_ATTR util_parse_command_qstring(const char *commandrcvd, const char *command, const char *message, char **val)
{
	int res;
	char param[32];
	struct jsonparse_state state;
	
	res = processCommandParam(commandrcvd, command, message, param, sizeof(param));
	
	if(-1 == res)
		return FALSE;
	
	if(2 == res){
		*val = util_strdup(param);
	}
	else{
		*val = NULL;
	}
	return TRUE;

}


