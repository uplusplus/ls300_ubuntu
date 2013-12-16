/*
 * session.h
 *
 *  Created on: Nov 07, 2013
 *      Author: uplusplus
 */
#include "mongoose.h"

#define authorize_url  "/authorize"
#define login_url      "/login.html"
#define MAX_USER_LEN  20

// Describes web session.
struct session {
	char session_id[33]; // Session ID, must be unique
	char random[20]; // Random data used for extra user validation
	char user[MAX_USER_LEN]; // Authenticated user
	time_t expire; // Expiration timestamp, UTC
};

// Redirect user to the login form. In the cookie, store the original URL
// we came from, so that after the authorization we could redirect back.
void redirect_to_login(struct mg_connection *conn,
		const struct mg_request_info *request_info);

// A handler for the /authorize endpoint.
// Login page form sends user name and password to this endpoint.
int authorize(struct mg_connection *conn,
		const struct mg_request_info *request_info);
struct session* authorize_ex(struct mg_connection *conn,
		const struct mg_request_info *request_info);

// Return 1 if request is authorized, 0 otherwise.
int is_authorized(const struct mg_connection *conn,
		const struct mg_request_info *request_info);
